"""이미지 픽셀 피팅 없이 월드 카메라 pose로 하는 3D 투영 테스트."""

import unittest

import numpy as np

from ac_validation.projection import (RenderCameraPose, RenderCarPose,
                                      box_corners_render_car, opp_box_yaw,
                                      project_points_render_camera)


class ProjectionTests(unittest.TestCase):
    def test_optical_axis_projects_to_principal_point(self):
        cam = RenderCameraPose(np.zeros(3), np.array([1.0, 0.0, 0.0]),
                               np.array([0.0, 0.0, 1.0]))
        uv, depth = project_points_render_camera(np.array([[10.0, 0.0, 0.0]]),
                                                 cam, 1000.0, 960.0, 460.0)
        np.testing.assert_allclose(uv[0], [960.0, 460.0])
        self.assertEqual(depth[0], 10.0)

    def test_body_yaw_has_priority_over_velocity_direction(self):
        self.assertEqual(opp_box_yaw({"yaw_body": 0.7, "opp_yaw": -1.2,
                                      "opp_speedKMH": 200.0}), 0.7)

    def test_render_car_box_uses_full_body_axes(self):
        # 90° nose-up synthetic pose: local forward extent becomes world height extent.
        car = RenderCarPose(np.zeros(3), np.array([0.0, 0.0, 1.0]),
                            np.array([-1.0, 0.0, 0.0]), np.zeros(3),
                            np.array([4.0, 2.0, 1.0]))
        corners = box_corners_render_car(car)
        self.assertAlmostEqual(np.ptp(corners[:, 2]), 4.0)
        self.assertAlmostEqual(np.ptp(corners[:, 0]), 1.0)


if __name__ == "__main__":
    unittest.main()
