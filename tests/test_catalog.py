import copy
import unittest
from dexbench.catalog import validate_tasks


def task():
    return {'id': 'G01', 'category': 'grasp', 'name': 'demo', 'status': 'selected', 'source': {'kind': 'original', 'url': 'https://example.org/task',
                                                                                               'revision': None}, 'original_success': None, 'assets': None, 'baseline_evidence': None, 'unilab': {'integrated': False}}


class CatalogTests(unittest.TestCase):
    def test_selected_may_have_unknowns(
        self): self.assertEqual(validate_tasks([task()]), [])

    def test_duplicate_rejected(self): self.assertTrue(
        validate_tasks([task(), task()]))

    def test_unknown_status_rejected(self):
        t = task()
        t['status'] = 'done'
        self.assertTrue(validate_tasks([t]))

    def test_baseline_requires_protocol_and_evidence(self):
        t = task()
        t['status'] = 'baseline_verified'
        self.assertTrue(validate_tasks([t]))

    def test_original_requires_source_url(self):
        t = task()
        t['source']['url'] = None
        self.assertTrue(validate_tasks([t]))

    def test_internal_may_have_no_public_url(self):
        t = task()
        t['source']['kind'] = 'internal'
        t['source']['url'] = None
        self.assertEqual(validate_tasks([t]), [])

    def test_integration_requires_evidence(self):
        t = task()
        t['unilab']['integrated'] = True
        self.assertTrue(validate_tasks([t]))


class PromotionTests(unittest.TestCase):
    def test_review_requires_scoring_code_location(self):
        t = task()
        t.update(status='source_reviewed', original_success='goal')
        t['source']['revision'] = 'abc'
        self.assertTrue(validate_tasks([t]))

    def test_scene_requires_mapping_and_reset_evidence(self):
        t = task()
        t.update(
            status='scene_ready',
            original_success='goal',
            assets=['asset'])
        t['source'].update(revision='abc', scoring_code_path='score.py')
        self.assertTrue(validate_tasks([t]))
        t['robot_mapping'] = {'file': 'mapping.json'}
        self.assertTrue(validate_tasks([t]))
        t['scene_evidence'] = {'load': 'run1', 'reset': 'run1'}
        self.assertEqual(validate_tasks([t]), [])
