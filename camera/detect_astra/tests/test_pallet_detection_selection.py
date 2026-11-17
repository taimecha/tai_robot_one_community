"""Candidate ambiguity must use the same confidence gate as acquisition."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pallet_acquisition import AcquisitionSession
from pallet_detection_selection import unique_pallet_indices, MIN_PALLET_CONFIDENCE


class PalletSelectionTests(unittest.TestCase):
    names = {0: 'emty pallet', 1: 'pallets', 2: 'Box'}

    def test_recorded_background_false_positive_does_not_veto_real_pallet(self):
        # Live Astra capture 2026-11-17: pallet on floor and a bag on the desk.
        boxes = [[293.058, 281.813, 436.269, 381.260],
                 [375.142, 9.208, 465.984, 61.613]]
        self.assertEqual(unique_pallet_indices(boxes, [0, 1], [.9364, .7163], self.names), [0])

    def test_two_qualified_separate_pallets_remain_ambiguous(self):
        self.assertEqual(unique_pallet_indices([[0, 0, 100, 100], [150, 0, 250, 100]],
                         [0, 1], [.94, .80], self.names), [0, 1])

    def test_duplicate_labels_on_same_object_count_once(self):
        self.assertEqual(unique_pallet_indices([[0, 0, 100, 100], [1, 1, 99, 99]],
                         [0, 1], [.85, .93], self.names), [1])

    def test_box_class_does_not_count_as_pallet(self):
        self.assertEqual(unique_pallet_indices([[0, 0, 100, 100]], [2], [.99], self.names), [])

    def test_invalid_or_unqualified_candidates_cannot_seed(self):
        for score in (.7499, float('nan'), float('inf'), 1.01, -1.):
            with self.subTest(score=score):
                self.assertEqual(unique_pallet_indices([[0, 0, 100, 100]], [0], [score], self.names), [])
        for box in ([0, 0, 0, 100], [0, float('nan'), 100, 100]):
            self.assertEqual(unique_pallet_indices([box], [0], [.99], self.names), [])

    def test_selection_and_acquisition_agree_at_confidence_boundary(self):
        box = [0, 0, 100, 100]
        for score, eligible in ((MIN_PALLET_CONFIDENCE, True),
                                (MIN_PALLET_CONFIDENCE-1e-5, False)):
            with self.subTest(score=score):
                indices = unique_pallet_indices([box], [0], [score], self.names)
                session = AcquisitionSession()
                session.start(0., 100.)
                session.detection(100., .1, 100.1, True, score, box, 'emty pallet')
                self.assertEqual(bool(indices), eligible)
                self.assertEqual(bool(session.detections), eligible)


if __name__ == '__main__':
    unittest.main()
