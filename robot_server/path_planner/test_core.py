"""安全抓取航路规划的离线测试。"""

import unittest

from path_planner.core import (PlanError, build_grasp_plan, densify_waypoints,
                               held_joint_configuration)


class SafeGraspPlanTest(unittest.TestCase):
    def test_plan_lifts_before_horizontal_translation(self):
        plan = build_grasp_plan(
            (0.2, 0.1, 0.8), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.9),
            table_z_m=0.7)
        names = [point.name for point in plan]
        self.assertEqual(names, [
            'current', 'lift_before_translation', 'above_pregrasp',
            'pregrasp', 'lift_after_grasp'])
        self.assertGreater(plan[1].position[2], plan[0].position[2])
        self.assertGreaterEqual(plan[3].position[2], 0.8)
        self.assertEqual(plan[2].position[:2], plan[3].position[:2])
        self.assertEqual(plan[3].position[:2], plan[4].position[:2])
        self.assertAlmostEqual(plan[4].position[2] - plan[3].position[2], 0.15)
        self.assertNotIn('grasp', names)
        self.assertAlmostEqual(plan[1].position[2], 0.94)

    def test_travel_preserves_higher_current_height(self):
        plan = build_grasp_plan(
            (0.2, 0.1, 1.05), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.9),
            table_z_m=0.7)
        above_pregrasp = next(point for point in plan if point.name == 'above_pregrasp')
        self.assertAlmostEqual(above_pregrasp.position[2], 1.05)

    def test_plan_rejects_closing_point_below_table_clearance(self):
        with self.assertRaisesRegex(PlanError, '低于桌面安全高度'):
            build_grasp_plan(
                (0.2, 0.1, 0.8), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.78),
                table_z_m=0.7)

    def test_plan_rejects_invalid_clearance(self):
        with self.assertRaises(PlanError):
            build_grasp_plan(
                (0.2, 0.1, 0.8), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.9),
                table_z_m=0.7, table_clearance_mm=0)

    def test_current_point_may_be_below_travel_height_before_vertical_lift(self):
        plan = build_grasp_plan(
            (0.2, 0.1, 0.774), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.9),
            table_z_m=0.7, table_clearance_mm=100, start_clearance_mm=20)
        self.assertEqual(plan[0].position[2], 0.774)
        self.assertGreaterEqual(plan[1].position[2], 0.8)

    def test_densify_limits_each_executed_step(self):
        plan = build_grasp_plan(
            (0.2, 0.1, 0.8), (0.0, 0.0, 0.0, 1.0), (0.6, 0.2, 0.9),
            table_z_m=0.7)
        dense = densify_waypoints(plan, 0.01)
        self.assertGreater(len(dense), len(plan))
        for first, second in zip(dense, dense[1:]):
            self.assertLessEqual(
                sum((a - b) ** 2 for a, b in zip(first.position, second.position)) ** 0.5,
                0.010000001)

    def test_final_stages_can_run_without_intermediate_wait_points(self):
        plan = build_grasp_plan(
            (0.2, 0.1, 0.8), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.9),
            table_z_m=0.7)
        dense = densify_waypoints(
            plan, 0.01, direct_targets=('pregrasp', 'lift_after_grasp'))
        names = [point.name for point in dense]
        self.assertTrue(any(name.startswith('above_pregrasp[') for name in names))
        self.assertEqual(names[-3:], ['above_pregrasp', 'pregrasp', 'lift_after_grasp'])
        self.assertFalse(any(name.startswith('pregrasp[') or
                             name.startswith('grasp[') or
                             name.startswith('lift_after_grasp[') for name in names))

    def test_closing_point_remains_when_above_point_has_same_position(self):
        plan = build_grasp_plan(
            (0.2, 0.1, 0.8), (0.0, 0.0, 0.0, 1.0), (0.4, 0.2, 0.9),
            table_z_m=0.7, overhead_mm=30.0, approach_height_mm=30.0)
        self.assertEqual(plan[-2].name, 'pregrasp')
        self.assertEqual(plan[-2].position, plan[-3].position)

    def test_joint_target_keeps_body_and_other_arm_fixed(self):
        hold = [float(index) for index in range(22)]
        feedback = [float(index + 100) for index in range(22)]
        left = held_joint_configuration(hold, feedback, 'left')
        self.assertEqual(left[:8], hold[:8])
        self.assertEqual(left[8:15], feedback[8:15])
        self.assertEqual(left[15:], hold[15:])
        right = held_joint_configuration(hold, feedback, 'right')
        self.assertEqual(right[:15], hold[:15])
        self.assertEqual(right[15:], feedback[15:])


if __name__ == '__main__':
    unittest.main()
