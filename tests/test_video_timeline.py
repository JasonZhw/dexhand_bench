import tempfile
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'integrations/bench2dex'))
from video_record import VideoTimeline

class VideoTimelineTests(unittest.TestCase):
    def test_frame_state_mapping_and_receipt(self):
        with tempfile.TemporaryDirectory() as tmp:
            timeline=VideoTimeline(Path(tmp),20,3)
            self.assertFalse(timeline.due(1))
            self.assertTrue(timeline.due(3))
            timeline.add(3,'frame_000000.png')
            self.assertEqual(timeline.rows[0]['state_t_s'],.05)
            self.assertEqual(timeline.rows[0]['physics_step'],3)
            self.assertTrue(timeline.finish(3)['complete'])
    def test_missing_frames_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertFalse(VideoTimeline(Path(tmp),20,3).finish(3)['complete'])
    def test_duplicate_step_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            timeline=VideoTimeline(Path(tmp),20,3)
            timeline.add(3,'a.png')
            with self.assertRaises(ValueError):timeline.add(3,'b.png')
            timeline.finish(3)
