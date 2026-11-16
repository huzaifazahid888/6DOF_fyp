# Synthetic fruit appearance model

The exported model is a trained multilayer perceptron with RGB input, hidden layers of 24 and 16 ReLU units, and five softmax output classes: background, leaf, ripe skin, unripe skin and visible defect. Runtime inference uses NumPy. It is not a renamed colour-threshold rule or a pretrained orchard detector.

Training uses 25,000 synthetic RGB samples with palette, brightness and channel-noise variation. Evaluation uses 7,500 independently sampled colours with a different seed. Seeds, confusion matrix and accuracy are stored in `appearance.json`. The initial held-out synthetic pixel accuracy is approximately 99.91%. This metric is **not** real-image fruit detection accuracy or an agricultural ripeness score.

The model detects appearance pixels. Depth sphere fitting, visibility checks, temporal consistency and LiDAR corroboration determine whether an object is accepted. A low-confidence or heavily occluded fruit is rejected. Ripeness assumes the chosen synthetic apple variety; other varieties, lighting spectra, diseased leaves, similar-coloured objects and real photographs are outside the evaluated distribution.

Reproduction requires Python, NumPy and scikit-learn:

```bash
python3 robotics_lab/src/pieas_orchard/training/train_appearance.py
```

Training is optional; the exported weights are included. Real deployment requires labelled orchard data, image-level and tree-level train/test splits, detector evaluation, calibrated ripeness labels and held-out evaluation under occlusion. No real-fruit generalisation is claimed.
