from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
r=root/'runs/external/apple4_video_01/recording'
rows=[json.loads(l) for l in (r/'states.jsonl').read_text().splitlines()]
initial=rows[0];steps=[x for x in rows if x['phase']=='replay']
t=np.array([x['t_s'] for x in steps]);pose=np.array([x['object_pose_wxyz'][:3] for x in steps]);d=pose-np.array(initial['object_pose_wxyz'][:3])
q=np.array([x['joint_positions'] for x in steps]);target=np.array([x['command_targets'] for x in steps]);err=target-q
fig,ax=plt.subplots(2,1,figsize=(10,5.8),sharex=True,layout='constrained')
for i,label in enumerate(['x','y','z']):ax[0].plot(t,d[:,i]*1000,label=label)
ax[0].set(ylabel='Origin displacement (mm)',title='Selected object: obj_013_apple_4 (source clip manipulated object)');ax[0].legend(ncol=3);ax[0].grid(alpha=.2)
ax[1].plot(t,np.max(abs(err),axis=1),label='Max absolute joint target error')
ax[1].plot(t,np.sqrt(np.mean(err**2,axis=1)),label='Joint target error RMS')
ax[1].set(xlabel='Simulation time (s)',ylabel='Angle (rad)',title='Control tracking diagnostic; not physical accuracy');ax[1].legend();ax[1].grid(alpha=.2)
out=root.parent/'outputs/DexBench_周会技术报告_20261112/media'
fig.savefig(out/'replay_diagnostics.png',dpi=150)
