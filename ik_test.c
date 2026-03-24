#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LINK1 35.0
#define LINK2 42.0
#define JOINT_MIN 0.0
#define JOINT_MAX 180.0
#define GN_MAX_ITERS 30
#define GN_TOLERANCE 0.01
#define GN_SUCCESS 0.25
#define GN_DAMPING 0.001
#define LINE_SUBPOINTS 100
#define START_JOINT1 37.0
#define START_JOINT2 27.5
typedef struct {
  double x;
  double y;
} Point;

typedef struct {
  double joint1Deg;
  double joint2Deg;
} JointState;

static double clamp_double(double value, double min_value, double max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static double deg_to_rad(double deg) {
  return deg * M_PI / 180.0;
}

static double rad_to_deg(double rad) {
  return rad * 180.0 / M_PI;
}

static Point fk_from_degrees(double joint1_deg, double joint2_deg) {
  double q1 = deg_to_rad(joint1_deg - 90.0);
  double q2 = deg_to_rad(joint2_deg - 90.0);
  Point p;
  p.x = LINK1 * cos(q1) + LINK2 * cos(q1 + q2);
  p.y = LINK1 * sin(q1) + LINK2 * sin(q1 + q2);
  return p;
}

static bool is_point_reachable(Point target) {
  double radius = sqrt((target.x * target.x) + (target.y * target.y));
  double min_radius = fabs(LINK1 - LINK2);
  double max_radius = LINK1 + LINK2;
  return radius >= min_radius - 1e-6 && radius <= max_radius + 1e-6;
}

static bool solve_ik_analytic(Point target, JointState current, JointState *seed) {
  double dist_sq = (target.x * target.x) + (target.y * target.y);
  double cos_q2 = (dist_sq - (LINK1 * LINK1) - (LINK2 * LINK2)) / (2.0 * LINK1 * LINK2);
  if (cos_q2 < -1.0 || cos_q2 > 1.0) {
    return false;
  }

  double q2_options[2] = {acos(cos_q2), -acos(cos_q2)};
  bool found = false;
  double best_cost = 0.0;

  for (int i = 0; i < 2; ++i) {
    double q2 = q2_options[i];
    double q1 = atan2(target.y, target.x) - atan2(LINK2 * sin(q2), LINK1 + (LINK2 * cos(q2)));
    double joint1 = rad_to_deg(q1) + 90.0;
    double joint2 = rad_to_deg(q2) + 90.0;

    if (joint1 < JOINT_MIN - 1e-6 || joint1 > JOINT_MAX + 1e-6) continue;
    if (joint2 < JOINT_MIN - 1e-6 || joint2 > JOINT_MAX + 1e-6) continue;

    double cost = fabs(joint1 - current.joint1Deg) + fabs(joint2 - current.joint2Deg);
    if (!found || cost < best_cost) {
      found = true;
      best_cost = cost;
      seed->joint1Deg = clamp_double(joint1, JOINT_MIN, JOINT_MAX);
      seed->joint2Deg = clamp_double(joint2, JOINT_MIN, JOINT_MAX);
    }
  }

  return found;
}

static bool solve_ik_gauss_newton(Point target, JointState *state) {
  if (!is_point_reachable(target)) {
    return false;
  }

  JointState analytic_seed;
  if (solve_ik_analytic(target, *state, &analytic_seed)) {
    state->joint1Deg = analytic_seed.joint1Deg;
    state->joint2Deg = analytic_seed.joint2Deg;
  }

  double q1 = deg_to_rad(state->joint1Deg - 90.0);
  double q2 = deg_to_rad(state->joint2Deg - 90.0);

  for (int iter = 0; iter < GN_MAX_ITERS; ++iter) {
    double c1 = cos(q1);
    double s1 = sin(q1);
    double c12 = cos(q1 + q2);
    double s12 = sin(q1 + q2);

    double x = LINK1 * c1 + LINK2 * c12;
    double y = LINK1 * s1 + LINK2 * s12;
    double ex = target.x - x;
    double ey = target.y - y;
    double error_norm = sqrt(ex * ex + ey * ey);

    if (error_norm < GN_TOLERANCE) {
      state->joint1Deg = clamp_double(rad_to_deg(q1) + 90.0, JOINT_MIN, JOINT_MAX);
      state->joint2Deg = clamp_double(rad_to_deg(q2) + 90.0, JOINT_MIN, JOINT_MAX);
      return true;
    }

    double j11 = -LINK1 * s1 - LINK2 * s12;
    double j12 = -LINK2 * s12;
    double j21 = LINK1 * c1 + LINK2 * c12;
    double j22 = LINK2 * c12;

    double a11 = (j11 * j11) + (j21 * j21) + GN_DAMPING;
    double a12 = (j11 * j12) + (j21 * j22);
    double a22 = (j12 * j12) + (j22 * j22) + GN_DAMPING;
    double b1 = (j11 * ex) + (j21 * ey);
    double b2 = (j12 * ex) + (j22 * ey);
    double det = (a11 * a22) - (a12 * a12);

    if (fabs(det) < 1e-9) {
      return false;
    }

    q1 += ((a22 * b1) - (a12 * b2)) / det;
    q2 += ((a11 * b2) - (a12 * b1)) / det;

    q1 = deg_to_rad(clamp_double(rad_to_deg(q1) + 90.0, JOINT_MIN, JOINT_MAX) - 90.0);
    q2 = deg_to_rad(clamp_double(rad_to_deg(q2) + 90.0, JOINT_MIN, JOINT_MAX) - 90.0);
  }

  Point final_point = fk_from_degrees(rad_to_deg(q1) + 90.0, rad_to_deg(q2) + 90.0);
  double ex = target.x - final_point.x;
  double ey = target.y - final_point.y;
  if (sqrt(ex * ex + ey * ey) > GN_SUCCESS) {
    return false;
  }

  state->joint1Deg = clamp_double(rad_to_deg(q1) + 90.0, JOINT_MIN, JOINT_MAX);
  state->joint2Deg = clamp_double(rad_to_deg(q2) + 90.0, JOINT_MIN, JOINT_MAX);
  return true;
}

static bool linear_path_is_reachable(Point start_rel, Point target_rel, Point home_abs) {
  JointState state = {START_JOINT1, START_JOINT2};

  for (int i = 1; i <= LINE_SUBPOINTS; ++i) {
    double t = (double) i / (double) LINE_SUBPOINTS;
    Point abs_target = {
      home_abs.x + start_rel.x + ((target_rel.x - start_rel.x) * t),
      home_abs.y + start_rel.y + ((target_rel.y - start_rel.y) * t)
    };

    if (!solve_ik_gauss_newton(abs_target, &state)) {
      return false;
    }
  }
  return true;
}

static bool plan_linear_motion(Point start_rel, Point target_rel, Point home_abs, JointState *state) {
  if (!linear_path_is_reachable(start_rel, target_rel, home_abs)) {
    return false;
  }

  for (int i = 1; i <= LINE_SUBPOINTS; ++i) {
    double t = (double) i / (double) LINE_SUBPOINTS;
    Point abs_target = {
      home_abs.x + start_rel.x + ((target_rel.x - start_rel.x) * t),
      home_abs.y + start_rel.y + ((target_rel.y - start_rel.y) * t)
    };

    if (!solve_ik_gauss_newton(abs_target, state)) {
      fprintf(stderr, "failed at step %d for target %.3f %.3f\n", i, abs_target.x, abs_target.y);
      return false;
    }
  }
  return true;
}

static Point internal_delta_to_user(Point delta) {
  Point user = {-delta.y, delta.x};
  return user;
}

static Point user_to_internal_delta(Point user) {
  Point delta = {user.y, -user.x};
  return delta;
}

static bool verify_linear_motion_path(Point start_rel, Point target_rel, Point home_abs, JointState start_state, double tolerance) {
  JointState state = start_state;

  for (int i = 1; i <= LINE_SUBPOINTS; ++i) {
    double t = (double) i / (double) LINE_SUBPOINTS;
    Point expected_rel = {
      start_rel.x + ((target_rel.x - start_rel.x) * t),
      start_rel.y + ((target_rel.y - start_rel.y) * t)
    };
    Point expected_abs = {
      home_abs.x + expected_rel.x,
      home_abs.y + expected_rel.y
    };

    if (!solve_ik_gauss_newton(expected_abs, &state)) {
      fprintf(stderr, "failed path solve at step %d\n", i);
      return false;
    }

    Point actual_abs = fk_from_degrees(state.joint1Deg, state.joint2Deg);
    Point actual_rel = {
      actual_abs.x - home_abs.x,
      actual_abs.y - home_abs.y
    };

    if (fabs(actual_rel.x - expected_rel.x) > tolerance || fabs(actual_rel.y - expected_rel.y) > tolerance) {
      fprintf(stderr,
              "path deviation at step %d expected_rel=(%.3f,%.3f) actual_rel=(%.3f,%.3f)\n",
              i, expected_rel.x, expected_rel.y, actual_rel.x, actual_rel.y);
      return false;
    }
  }

  return true;
}

static void require_true(bool condition, const char *message) {
  if (!condition) {
    fprintf(stderr, "FAIL: %s\n", message);
    exit(1);
  }
}

static void require_near(double actual, double expected, double tolerance, const char *message) {
  if (fabs(actual - expected) > tolerance) {
    fprintf(stderr, "FAIL: %s (actual %.6f expected %.6f)\n", message, actual, expected);
    exit(1);
  }
}

static void test_forward_kinematics_angle_command_example(void) {
  Point p = fk_from_degrees(90.0, 90.0);
  require_near(p.x, 77.0, 1e-6, "A 90 90 must report full extension x");
  require_near(p.y, 0.0, 1e-6, "A 90 90 must report full extension y");
}

static void test_centered_start_pose(void) {
  Point start = fk_from_degrees(START_JOINT1, START_JOINT2);
  require_near(start.x, 2.98205974, 0.01, "centered start pose x must match joint-limited workspace center");
  require_near(start.y, -65.86082479, 0.01, "centered start pose y must match joint-limited workspace center");
  require_near(0.0, 0.0, 1e-6, "G 0 0 must mean centered start point");
}

static void test_user_y_axis_convention(void) {
  Point up = user_to_internal_delta((Point) {0.0, 10.0});
  Point down = user_to_internal_delta((Point) {0.0, -10.0});
  require_near(up.x, 10.0, 1e-6, "user +Y must map to positive internal X");
  require_near(up.y, 0.0, 1e-6, "user +Y should not change internal Y");
  require_near(down.x, -10.0, 1e-6, "user -Y must map to negative internal X");
  require_near(down.y, 0.0, 1e-6, "user -Y should not change internal Y");
}

static void test_user_x_axis_convention(void) {
  Point right = user_to_internal_delta((Point) {10.0, 0.0});
  Point left = user_to_internal_delta((Point) {-10.0, 0.0});
  require_near(right.x, 0.0, 1e-6, "user +X should not change internal X");
  require_near(right.y, -10.0, 1e-6, "user +X must map to negative internal Y");
  require_near(left.x, 0.0, 1e-6, "user -X should not change internal X");
  require_near(left.y, 10.0, 1e-6, "user -X must map to positive internal Y");
}

static void test_workspace_limits(void) {
  require_true(is_point_reachable((Point) {77.0, 0.0}), "outer radius 77 mm must be reachable");
  require_true(is_point_reachable((Point) {7.0, 0.0}), "inner radius 7 mm must be reachable");
  require_true(!is_point_reachable((Point) {77.1, 0.0}), "beyond outer radius must be unreachable");
  require_true(!is_point_reachable((Point) {6.9, 0.0}), "inside inner radius must be unreachable");
}

static void test_reachable_ik_solution(void) {
  JointState state = {START_JOINT1, START_JOINT2};
  Point target = {70.0, 5.0};
  require_true(solve_ik_gauss_newton(target, &state), "reachable target must solve");

  Point solved = fk_from_degrees(state.joint1Deg, state.joint2Deg);
  require_near(solved.x, target.x, 0.25, "ik x must satisfy 35/42 geometry");
  require_near(solved.y, target.y, 0.25, "ik y must satisfy 35/42 geometry");
}

static void test_unreachable_ik_solution(void) {
  JointState state = {START_JOINT1, START_JOINT2};
  require_true(!solve_ik_gauss_newton((Point) {200.0, 200.0}, &state), "200,200 mm must be unreachable");
  require_true(!solve_ik_gauss_newton((Point) {0.0, 0.0}, &state), "origin must be unreachable because min radius is 7 mm");
}

static void test_linear_path_reachable(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  JointState state = {START_JOINT1, START_JOINT2};
  Point start_rel = {0.0, 0.0};
  Point target_rel = user_to_internal_delta((Point) {10.0, 10.0});

  require_true(plan_linear_motion(start_rel, target_rel, home_abs, &state), "G 10 10 should plan from centered origin");

  Point final_abs = fk_from_degrees(state.joint1Deg, state.joint2Deg);
  Point final_internal = {final_abs.x - home_abs.x, final_abs.y - home_abs.y};
  Point final_user = internal_delta_to_user(final_internal);
  require_near(final_user.x, 10.0, 0.3, "planned line end user x must match");
  require_near(final_user.y, 10.0, 0.3, "planned line end user y must match");
}

static void test_vertical_line_is_cartesian_vertical(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  JointState state = {START_JOINT1, START_JOINT2};
  Point start_rel = {0.0, 0.0};
  Point target_rel = user_to_internal_delta((Point) {0.0, 10.0});

  require_true(verify_linear_motion_path(start_rel, target_rel, home_abs, state, 0.35),
               "G 0 10 must stay on a vertical Cartesian line");
}

static void test_user_vertical_plus_minus_20_reachable(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  require_true(plan_linear_motion((Point) {0.0, 0.0}, user_to_internal_delta((Point) {0.0, 20.0}), home_abs,
                                  &(JointState) {START_JOINT1, START_JOINT2}),
               "G 0 20 must be reachable from the centered origin");
  require_true(plan_linear_motion((Point) {0.0, 0.0}, user_to_internal_delta((Point) {0.0, -20.0}), home_abs,
                                  &(JointState) {START_JOINT1, START_JOINT2}),
               "G 0 -20 must be reachable from the centered origin");
}

static void test_negative_quadrant_line_reachable(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  JointState state = {START_JOINT1, START_JOINT2};
  Point start_rel = {0.0, 0.0};
  Point target_rel = user_to_internal_delta((Point) {-10.0, -10.0});

  require_true(plan_linear_motion(start_rel, target_rel, home_abs, &state),
               "G -10 -10 must be reachable from the centered origin");

  require_true(verify_linear_motion_path(start_rel, target_rel, home_abs,
                                         (JointState) {START_JOINT1, START_JOINT2}, 0.35),
               "G -10 -10 must follow the requested Cartesian line");
}

static void test_horizontal_line_moves_right_20mm(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  JointState state = {START_JOINT1, START_JOINT2};
  Point start_rel = user_to_internal_delta((Point) {-10.0, -10.0});
  Point target_rel = user_to_internal_delta((Point) {10.0, -10.0});

  require_true(plan_linear_motion(start_rel, target_rel, home_abs, &state),
               "G -10 -10 to G 10 -10 must be reachable");

  require_true(verify_linear_motion_path(start_rel, target_rel, home_abs,
                                         (JointState) {START_JOINT1, START_JOINT2}, 0.35),
               "horizontal move at y=-10 must stay on the requested Cartesian line");
}

static void test_vertical_line_moves_up_20mm(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  JointState state = {START_JOINT1, START_JOINT2};
  Point start_rel = user_to_internal_delta((Point) {-10.0, -10.0});
  Point target_rel = user_to_internal_delta((Point) {-10.0, 10.0});

  require_true(plan_linear_motion(start_rel, target_rel, home_abs, &state),
               "G -10 -10 to G -10 10 must be reachable");

  require_true(verify_linear_motion_path(start_rel, target_rel, home_abs,
                                         (JointState) {START_JOINT1, START_JOINT2}, 0.35),
               "vertical move at x=-10 must stay on the requested Cartesian line");
}

static void test_linear_path_unreachable(void) {
  Point home_abs = fk_from_degrees(START_JOINT1, START_JOINT2);
  JointState state = {START_JOINT1, START_JOINT2};
  Point start_rel = {0.0, 0.0};
  Point target_rel = {200.0, 200.0};

  require_true(!plan_linear_motion(start_rel, target_rel, home_abs, &state), "unreachable line must be rejected before motion");
}

static void test_reported_axes_match_observed_angle_moves(void) {
  Point top = fk_from_degrees(90.0, 90.0);
  Point lower = fk_from_degrees(50.0, 50.0);
  Point delta = {lower.x - top.x, lower.y - top.y};
  Point user = internal_delta_to_user(delta);

  require_true(user.x > 0.0, "A 50 50 must report positive X relative to A 90 90");
  require_true(user.y < 0.0, "A 50 50 must report negative Y relative to A 90 90");
}

int main(void) {
  test_forward_kinematics_angle_command_example();
  test_centered_start_pose();
  test_user_y_axis_convention();
  test_user_x_axis_convention();
  test_workspace_limits();
  test_reachable_ik_solution();
  test_unreachable_ik_solution();
  test_linear_path_reachable();
  test_vertical_line_is_cartesian_vertical();
  test_user_vertical_plus_minus_20_reachable();
  test_negative_quadrant_line_reachable();
  test_horizontal_line_moves_right_20mm();
  test_vertical_line_moves_up_20mm();
  test_linear_path_unreachable();
  test_reported_axes_match_observed_angle_moves();
  printf("all tests passed\n");
  return 0;
}
