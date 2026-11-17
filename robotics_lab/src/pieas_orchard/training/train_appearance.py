#!/usr/bin/env python3
"""Reproducible small learned baseline; synthetic palette labels only.

Training needs scikit-learn. Runtime inference uses NumPy and the exported JSON.
Held-out synthetic accuracy does NOT measure real orchard detection accuracy.
"""
import json
from pathlib import Path
import numpy as np
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import confusion_matrix, accuracy_score

LABELS=['background','leaf','ripe','unripe','defect']
# Multiple palettes + brightness/channel variation. Background includes bark/soil.
PALETTES=[[[.45,.48,.5],[.45,.30,.18],[.8,.8,.72],[.15,.2,.25],[.15,.35,.8]],
          [[.08,.30,.06],[.12,.42,.07],[.06,.22,.04]],
          [[.82,.055,.025],[.65,.025,.02],[.95,.13,.06]],
          [[.65,.82,.06],[.48,.70,.035],[.80,.85,.12]],
          [[.18,.035,.08],[.26,.04,.12],[.12,.015,.05]]]
def samples(seed,n):
    rng=np.random.default_rng(seed);xs=[];ys=[]
    for cls,palettes in enumerate(PALETTES):
        palette=np.array(palettes)
        base=palette[rng.integers(len(palette),size=n)]
        linear=np.clip(base*rng.uniform(.25,1.15,(n,1))+rng.normal(0,.006,(n,3)),0,1)
        # Cover both linear fixtures and the renderer's gamma-encoded RGB output.
        gamma=rng.choice([1.,1/2.2],size=(n,1))+rng.uniform(-.025,.025,(n,1))
        x=linear**gamma+rng.normal(0,.01,(n,3))
        xs.append(np.clip(x,0,1));ys.extend([cls]*n)
    return np.vstack(xs),np.array(ys)
def train(output):
    x,y=samples(1701,5000);tx,ty=samples(2903,1500)
    model=MLPClassifier(hidden_layer_sizes=(24,16),max_iter=250,random_state=73,
                        early_stopping=True,n_iter_no_change=15)
    model.fit(x,y);pred=model.predict(tx)
    data={'labels':LABELS,'weights':[w.tolist() for w in model.coefs_],
          'biases':[b.tolist() for b in model.intercepts_],
          'training_seed':1701,'test_seed':2903,'synthetic_accuracy':accuracy_score(ty,pred),
          'confusion_matrix':confusion_matrix(ty,pred).tolist(),
          'scope':'Synthetic appearance segmentation, not real-world ripeness certification.'}
    Path(output).write_text(json.dumps(data,indent=2)+'\n');print(json.dumps({k:v for k,v in data.items() if k not in ['weights','biases']},indent=2))
if __name__=='__main__':
    train(Path(__file__).resolve().parents[1]/'models/appearance.json')

