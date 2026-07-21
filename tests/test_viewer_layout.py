import matplotlib
import unittest

matplotlib.use("Agg")

from ac_validation.viewer import _bbox_row_visible, _layout_image_labels


class ViewerLayoutTests(unittest.TestCase):
    def test_image_labels_do_not_overlap_and_stay_in_frame(self):
        labels = [(1, 460, 270), (2, 462, 271), (3, 465, 269),
                  (4, 461, 274), (5, 459, 268)]
        result = _layout_image_labels(labels)

        positions = [pos for _, _, pos in result]
        self.assertTrue(all(14 <= x <= 1906 and 12 <= y <= 933
                            for x, y in positions))
        for i, (x1, y1) in enumerate(positions):
            for x2, y2 in positions[i + 1:]:
                self.assertTrue(abs(x1 - x2) >= 24 or abs(y1 - y2) >= 20)

    def test_camera_bbox_excludes_retired_and_invalid_opponents(self):
        self.assertTrue(_bbox_row_visible(
            {"present": True, "retired": False, "valid": True}))
        self.assertFalse(_bbox_row_visible(
            {"present": True, "retired": True, "valid": False}))
        self.assertFalse(_bbox_row_visible(
            {"present": True, "retired": False, "valid": False}))
        self.assertFalse(_bbox_row_visible(
            {"present": False, "retired": False, "valid": True}))


if __name__ == "__main__":
    unittest.main()
