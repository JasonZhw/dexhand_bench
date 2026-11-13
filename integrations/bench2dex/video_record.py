"""Frame timestamps match post-step state records; rendering adds no physics steps."""
import json
from pathlib import Path

class VideoTimeline:
    def __init__(self, directory, fps, stride):
        self.directory=Path(directory)
        self.fps=fps
        self.stride=stride
        self.rows=[]
        self.stream=(self.directory/'video_frames.jsonl').open('x',buffering=1)
    def due(self, step):
        return step > 0 and step % self.stride == 0
    def add(self, step, filename):
        expected=(len(self.rows)+1)*self.stride
        if step != expected:
            raise ValueError('Nonconsecutive captured physics step')
        row={'frame_index':len(self.rows),'physics_step':step,
             'state_t_s':step/(self.fps*self.stride),'file':filename}
        self.stream.write(json.dumps(row)+'\n');self.rows.append(row)
    def finish(self, total_steps):
        self.stream.close()
        receipt={'complete':len(self.rows)==total_steps//self.stride and len(self.rows)>0,
                 'frames':len(self.rows),'fps':self.fps,'stride':self.stride,
                 'capture':'post-step RGB; no additional physics step for rendering',
                 'timing_note':'Includes rendering and PNG encoding; not throughput evidence'}
        (self.directory/'video_receipt.json').write_text(json.dumps(receipt,indent=2))
        return receipt
