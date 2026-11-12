"""Sensor-only shape classification. No Gazebo names, class IDs or object poses."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def sensor_transform(config, optical=False):
    r = Rotation.from_euler('xyz', config['rpy']).as_matrix()
    if optical:
        r = r @ Rotation.from_euler('xyz', [-np.pi/2,0,-np.pi/2]).as_matrix()
    return r, np.array(config['xyz'])


def project(pixels, depths, intrinsic, rotation, translation):
    """Optical frame: X right, Y down, Z forward. Depth is Z, not ray length."""
    fx, fy, cx, cy = intrinsic
    xyz = np.column_stack(((pixels[:,0]-cx)*depths/fx,
                           (pixels[:,1]-cy)*depths/fy, depths))
    return xyz @ rotation.T + translation


def classify(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if area < 60 or perimeter <= 0:
        return None
    rect = cv2.minAreaRect(contour)
    a,b = rect[1]
    if min(a,b) < 5:
        return None
    aspect = max(a,b)/min(a,b)
    fill = area/(a*b)
    circularity = 4*np.pi*area/perimeter**2
    polygon = cv2.approxPolyDP(contour, 0.025*perimeter, True)
    if len(polygon) == 4 and fill > 0.87:
        if aspect > 1.5 and aspect < 2.5:
            return 'rectangle', min(1.0,fill)
        if aspect < 1.2:
            return 'cube', min(1.0,fill)
    if aspect < 1.2 and circularity > 0.82 and len(polygon) >= 6:
        return 'cylinder', float(circularity)
    return None


def detect(rgb, depth, intrinsic, lidar_world, c):
    """Return candidates with camera XYZ, physical dimensions and LiDAR support."""
    if depth.shape != rgb.shape[:2]:
        raise ValueError('RGB/depth resolution mismatch; aligned RGBD images required.')
    r,t = sensor_transform(c['camera'], optical=True)
    v,u = np.indices(depth.shape)
    pixels = np.column_stack((u.ravel(),v.ravel()))
    world = project(pixels, depth.ravel(), intrinsic, r,t).reshape((*depth.shape,3))
    p,b = c['perception'],c['belt']
    h = world[:,:,2]-b['top']
    hsv = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    # A controlled green foreground separates parts from the arm/belt. All three
    # classes have the SAME colour; the silhouette determines the class.
    mask = (np.isfinite(depth) & (depth>0) &
            (np.abs(world[:,:,0]-b['x'])<p['roi_half_x']) &
            (np.abs(world[:,:,1]-b['stop_y'])<p['roi_half_y']) &
            (h>p['minimum_height']) & (h<p['maximum_height']) &
            (hsv[:,:,0]>35) & (hsv[:,:,0]<90) & (hsv[:,:,1]>80)).astype(np.uint8)*255
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    results=[]
    for contour in contours:
        category=classify(contour)
        if category is None:continue
        shape,confidence=category
        filled=np.zeros_like(mask);cv2.drawContours(filled,[contour],-1,255,-1)
        interior=cv2.erode(filled,np.ones((3,3),np.uint8))>0
        good=interior & np.isfinite(depth) & (depth>0)
        if good.sum()<30:continue
        top=float(np.median(world[:,:,2][good]))
        median_depth=float(np.median(depth[good]))
        m=cv2.moments(contour)
        centre_pixel=np.array([[m['m10']/m['m00'],m['m01']/m['m00']]])
        centre=project(centre_pixel,np.array([median_depth]),intrinsic,r,t)[0]
        footprint=project(contour[:,0,:].astype(float),np.full(len(contour),median_depth),intrinsic,r,t)[:,:2].astype(np.float32)
        rect=cv2.minAreaRect(footprint)
        dimensions=list(rect[1])
        yaw=np.deg2rad(rect[2])
        if dimensions[0]<dimensions[1]:
            dimensions.reverse();yaw+=np.pi/2
        height=top-b['top']
        # Reject clipped/merged silhouettes or objects outside our known size range.
        if not (0.018<min(dimensions)<0.045 and max(dimensions)<0.065 and 0.02<height<0.045):continue
        cloud=np.asarray(lidar_world).reshape(-1,3)
        cloud=cloud[np.isfinite(cloud).all(axis=1)]
        near=cloud[(np.abs(cloud[:,0]-centre[0])<max(dimensions)/2+0.006) &
                   (np.abs(cloud[:,1]-centre[1])<max(dimensions)/2+0.006) &
                   (cloud[:,2]>b['top']+p['minimum_height']) &
                   (cloud[:,2]<b['top']+p['maximum_height'])]
        if len(near)<p['minimum_lidar_points']:continue
        lidar_top=float(np.median(near[:,2]))
        if abs(lidar_top-top)>p['lidar_height_tolerance']:continue
        results.append({'shape':shape,'confidence':float(confidence),
                        'xyz':[float(centre[0]),float(centre[1]),top-height/2],
                        'top_z':top,'size':[float(dimensions[0]),float(dimensions[1]),height],
                        'yaw':float(yaw),'lidar_points':len(near),'lidar_top':lidar_top})
    return results,mask


class Stability:
    """Require consecutive independent sensor frames with nearly stationary XYZ."""
    def __init__(self,frames):
        self.frames=frames;self.previous=None;self.count=0
    def update(self,candidates):
        if len(candidates)!=1:
            self.previous=None;self.count=0;return None
        value=candidates[0]
        same=self.previous is not None and value['shape']==self.previous['shape'] and np.linalg.norm(
            np.array(value['xyz'])-self.previous['xyz'])<0.002
        self.count=self.count+1 if same else 1
        self.previous=value
        return value if self.count>=self.frames else None
