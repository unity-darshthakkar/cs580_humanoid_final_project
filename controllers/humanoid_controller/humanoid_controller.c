/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* CS580 NAO Basketball: keyboard navigation, hands, and squat/stand motions. */
#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <math.h>
#include <webots/utils/motion.h>

#define PHALANX_MAX 8
#define HAND_OPEN_ANGLE 0.96
#define HAND_CLOSED_ANGLE 0.0
#define HAND_CLOSE_SECONDS 1.8
/* 0.96=open, 0.0=full fist. 0.25 is a firmer partial-grip test. */
#define HAND_CARRY_ANGLE 0.25
#define HAND_GRIP_SETTLE_SECONDS 1.00

/* Pickup is assisted (kinematic follow), not a friction-only grasp. */
#define GRASP_MAX_DISTANCE 0.43
#define RELEASE_UP_SPEED 0.30
#define RELEASE_FORWARD_SPEED 0.35

static WbDeviceTag left_phalanx[PHALANX_MAX];
static WbDeviceTag right_phalanx[PHALANX_MAX];

/* Smooth, time-stepped finger closing rather than an instant fist. */
static double hand_angle = HAND_OPEN_ANGLE;
static double hand_target = HAND_OPEN_ANGLE;

static WbMotionRef forwards = NULL;
static WbMotionRef backwards = NULL;
static WbMotionRef backwards_carry = NULL;
static WbMotionRef side_step_left = NULL;
static WbMotionRef side_step_left_carry = NULL;
static WbMotionRef side_step_right = NULL;
static WbMotionRef side_step_right_carry = NULL;
static WbMotionRef turn_left = NULL;
static WbMotionRef turn_right = NULL;
static WbMotionRef squat = NULL;
static WbMotionRef stand_from_squat = NULL;
static WbMotionRef currently_playing = NULL;
static WbMotionRef shoot = NULL;

typedef enum {
  STANDING,
  SQUATTING,
  SQUATTED,
  PREPARING_GRIP,
  STANDING_UP
} PostureState;

static PostureState posture = STANDING;
static double grip_settle_remaining = 0.0;
static int grip_only_test = 0;

/* Supervisor state; ball must have DEF BASKETBALL in the Webots world. */
static WbNodeRef robot_node = NULL;
static WbNodeRef ball_node = NULL;
static WbFieldRef ball_translation = NULL;
static int ball_attached = 0;
/* Permanent visual-only basketball defined ONCE in the .wbt world.
   Never create/remove scene nodes during controller execution. */
static WbNodeRef carry_visual = NULL;
static WbFieldRef carry_visual_translation = NULL;

static double ball_offset_local[3] = {0.0, 0.0, 0.0};
static int invalid_pose_reported = 0;

static int finite_vec(const double *v, int count) {
  if (!v) return 0;
  for (int i = 0; i < count; ++i)
    if (!isfinite(v[i]) || fabs(v[i]) > 1000.0) return 0;
  return 1;
}

/* The orientation is a row-major local-to-world rotation matrix. */
static void local_to_world(const double *r, const double local[3], double world[3]) {
  for (int i = 0; i < 3; ++i)
    world[i] = r[3*i] * local[0] + r[3*i+1] * local[1] + r[3*i+2] * local[2];
}

static void world_to_local(const double *r, const double world[3], double local[3]) {
  for (int i = 0; i < 3; ++i)
    local[i] = r[i] * world[0] + r[3+i] * world[1] + r[6+i] * world[2];
}

static void initialize_supervisor_grasp(void) {
  if (!wb_robot_get_supervisor()) {
    fprintf(stderr, "SUPERVISOR NOT ACTIVE: confirm NAO supervisor TRUE and reopen the saved world.\n");
    return;
  }
  robot_node = wb_supervisor_node_get_self();
  ball_node = wb_supervisor_node_get_from_def("BASKETBALL");
  if (!robot_node || !ball_node) {
    fprintf(stderr, "GRASP SETUP ERROR: enable supervisor TRUE on NAO and set ball DEF to BASKETBALL.\n");
    return;
  }
  ball_translation = wb_supervisor_node_get_field(ball_node, "translation");
  if (!ball_translation) {
    fprintf(stderr, "GRASP SETUP ERROR: basketball translation field inaccessible.\n");
    ball_node = NULL;
    return;
  }
  printf("Supervisor grasp ready: NAO and DEF BASKETBALL detected.\n");
}

/* Calculate where a visually carried basketball belongs in world coordinates. */
static int get_carried_position(double target[3]) {
  const double *rp = wb_supervisor_node_get_position(robot_node);
  const double *r = wb_supervisor_node_get_orientation(robot_node);
  if (!finite_vec(rp, 3) || !finite_vec(r, 9) || !finite_vec(ball_offset_local, 3)) {
    if (!invalid_pose_reported) {
      fprintf(stderr, "INVALID NAO POSE: refusing to move basketball with NaN/Inf. Reset simulation; inspect robot physics.\n");
      invalid_pose_reported = 1;
    }
    return 0;
  }
  double rotated[3];
  local_to_world(r, ball_offset_local, rotated);
  for (int i = 0; i < 3; ++i) target[i] = rp[i] + rotated[i];
  return finite_vec(target, 3);
}

/* World contains one permanent, non-physical Transform with this DEF.
   It begins at an offstage location and stays in the Scene Tree. */
static int locate_carry_visual(void) {
  carry_visual = wb_supervisor_node_get_from_def("CARRIED_BASKETBALL_VISUAL");
  if (!carry_visual) {
    fprintf(stderr, "MISSING CARRIED_BASKETBALL_VISUAL: add the static Transform to the .wbt world (see README).\n");
    return 0;
  }
  carry_visual_translation = wb_supervisor_node_get_field(carry_visual, "translation");
  if (!carry_visual_translation) {
    fprintf(stderr, "Invalid carry visual: translation field not found.\n");
    return 0;
  }
  return 1;
}

/* These positions are intentionally far from the court and robot.
   Keep the physics-enabled Ball far from the camera while carrying. */
/* Park ON the floor, inside the 10x10 RectangleArena. A dynamic physics body
   at (200,200,-5) can fall indefinitely and destabilize Webots/Ode. */
static const double PHYSICAL_BALL_PARK[3] = {4.5, 4.5, 0.065};
static const double VISUAL_BALL_PARK[3] = {205.0, 205.0, -5.0};

static void park_physical_ball(void) {
  if (!ball_node || !ball_translation) return;
  wb_supervisor_field_set_sf_vec3f(ball_translation, PHYSICAL_BALL_PARK);
  const double zero[6] = {0, 0, 0, 0, 0, 0};
  wb_supervisor_node_set_velocity(ball_node, zero);
  wb_supervisor_node_reset_physics(ball_node);
}

/* Enforce a proximity gate so a distant ball cannot be picked up. */
static int attach_ball(void) {
  if (ball_attached) return 1;
  if (!robot_node || !ball_node || !ball_translation || !carry_visual_translation) {
    fprintf(stderr, "Cannot attach basketball: Supervisor or visual setup incomplete.\n");
    return 0;
  }
  const double *rp = wb_supervisor_node_get_position(robot_node);
  const double *bp = wb_supervisor_node_get_position(ball_node);
  const double *r = wb_supervisor_node_get_orientation(robot_node);
  if (!finite_vec(rp, 3) || !finite_vec(bp, 3) || !finite_vec(r, 9)) {
    fprintf(stderr, "Grasp rejected: invalid NAO/ball pose. Reset physics or simulation.\n");
    return 0;
  }

  double delta[3] = {bp[0]-rp[0],bp[1]-rp[1],bp[2]-rp[2]};
  const double dist = sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
  world_to_local(r, delta, ball_offset_local);
  if (dist > GRASP_MAX_DISTANCE || ball_offset_local[0] < 0.03 ||
      ball_offset_local[0] > 0.36 || fabs(ball_offset_local[1]) > 0.19) {
    fprintf(stderr, "Grasp not engaged: ball isn't in front of both hands (distance %.3f m, local %.3f %.3f %.3f).\n",
            dist, ball_offset_local[0], ball_offset_local[1], ball_offset_local[2]);
    return 0;
  }

  double target[3];
  if (!get_carried_position(target)) {
    fprintf(stderr, "Cannot compute carried basketball position.\n");
    return 0;
  }
  /* A persistent visual (no collision, no physics) is shown in the hands.
     Physical ball is parked stably on the court far from NAO, not deleted. */
  park_physical_ball();
  wb_supervisor_field_set_sf_vec3f(carry_visual_translation, target);
  ball_attached = 1;
  printf("Assisted grasp ENGAGED (collision-free carry visual; offset %.3f %.3f %.3f).\n",
         ball_offset_local[0], ball_offset_local[1], ball_offset_local[2]);
  return 1;
}

/* Only visual geometry moves while holding; no physics updates or collisions. */
static void follow_attached_ball(void) {
  if (!ball_attached || !carry_visual_translation || !robot_node) return;
  double target[3];
  if (get_carried_position(target))
    wb_supervisor_field_set_sf_vec3f(carry_visual_translation, target);
}

static void release_ball(int launch) {
  if (!ball_attached) {
    printf("Basketball isn't attached.\n");
    return;
  }
  double target[3];
  if (!get_carried_position(target)) {
    fprintf(stderr, "Release position unavailable; keeping assisted grasp active.\n");
    return;
  }
  /* Put the physical ball just forward of the fingers to minimize overlap
     with NAO's hands when normal rigid-body collisions resume. */
  const double *r = wb_supervisor_node_get_orientation(robot_node);
  double forward_world[3] = {1.0, 0.0, 0.0};
  if (r) {
    const double forward_local[3] = {1.0, 0.0, 0.0};
    local_to_world(r, forward_local, forward_world);
  }
  for (int i = 0; i < 3; ++i) target[i] += 0.12 * forward_world[i];
  target[2] += 0.035; /* Z is the up direction in this world. */

  /* Do not call wb_supervisor_node_remove: permanent visual is parked instead. */
  wb_supervisor_field_set_sf_vec3f(carry_visual_translation, VISUAL_BALL_PARK);
  if (!finite_vec(target, 3) || !finite_vec(forward_world, 3)) {
    fprintf(stderr, "Release canceled: invalid ball position.\n");
    return;
  }
  wb_supervisor_field_set_sf_vec3f(ball_translation, target);
  wb_supervisor_node_reset_physics(ball_node);
  const double velocity[6] = {
    launch ? RELEASE_FORWARD_SPEED * forward_world[0] : 0.0,
    launch ? RELEASE_FORWARD_SPEED * forward_world[1] : 0.0,
    launch ? RELEASE_FORWARD_SPEED * forward_world[2] + RELEASE_UP_SPEED : 0.0,
    0.0, 0.0, 0.0
  };
  wb_supervisor_node_set_velocity(ball_node, velocity);
  ball_attached = 0;
  hand_target = HAND_OPEN_ANGLE;
  printf(launch ? "Assisted grasp released (test velocity).\n" : "Assisted grasp released (drop).\n");
}


static void load_motion_files(void) {
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  backwards_carry = wbu_motion_new("../../motions/BackwardsCarry.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_left_carry = wbu_motion_new("../../motions/SideStepLeftCarry.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  side_step_right_carry = wbu_motion_new("../../motions/SideStepRightCarry.motion");
  turn_left = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right = wbu_motion_new("../../motions/TurnRight60.motion");
  squat = wbu_motion_new("../../motions/Squat.motion");
  stand_from_squat = wbu_motion_new("../../motions/StandFromSquat.motion");
  shoot = wbu_motion_new("../../motions/Shoot.motion");
  if (!backwards_carry || !side_step_left_carry || !side_step_right_carry)
    fprintf(stderr, "Carry navigation motion missing; copy *Carry.motion files to ../../motions/\n");
  if (!squat || !stand_from_squat)
    fprintf(stderr, "Squat motion file missing: check ../../motions/\n");
}

static int start_motion(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Motion not loaded; check ../../motions/\n");
    return 0;
  }

  if (currently_playing)
    wbu_motion_stop(currently_playing);

  wbu_motion_play(motion);
  currently_playing = motion;
  return 1;
}

static void initialize_hands(void) {
  for (int i = 0; i < PHALANX_MAX; ++i) {
    char name[32];
    snprintf(name, sizeof(name), "LPhalanx%d", i + 1);
    left_phalanx[i] = wb_robot_get_device(name);
    snprintf(name, sizeof(name), "RPhalanx%d", i + 1);
    right_phalanx[i] = wb_robot_get_device(name);
  }
}

static void set_hands_angle(double angle) {
  for (int i = 0; i < PHALANX_MAX; ++i) {
    const WbDeviceTag motors[2] = {left_phalanx[i], right_phalanx[i]};
    for (int hand = 0; hand < 2; ++hand) {
      WbDeviceTag motor = motors[hand];
      if (!motor)
        continue;
      double target = angle;
      const double min = wb_motor_get_min_position(motor);
      const double max = wb_motor_get_max_position(motor);
      if (target < min)
        target = min;
      if (target > max)
        target = max;
      wb_motor_set_position(motor, target);
    }
  }
}

/* Called once per simulation step. Keep moving gradually until the target is reached. */
static void update_hands(int time_step_ms) {
  const double step = HAND_OPEN_ANGLE * ((double)time_step_ms / 1000.0) / HAND_CLOSE_SECONDS;
  if (hand_angle < hand_target) {
    hand_angle += step;
    if (hand_angle > hand_target)
      hand_angle = hand_target;
  } else if (hand_angle > hand_target) {
    hand_angle -= step;
    if (hand_angle < hand_target)
      hand_angle = hand_target;
  }
  set_hands_angle(hand_angle);
}

static void update_posture(int time_step_ms) {
  /* On L, first close fingers partway, then start the stand-up motion. */
  if (posture == PREPARING_GRIP) {
    const double tolerance = 0.005;
    if (hand_angle <= HAND_CARRY_ANGLE + tolerance &&
        hand_angle >= HAND_CARRY_ANGLE - tolerance) {
      grip_settle_remaining -= (double)time_step_ms / 1000.0;
      if (grip_settle_remaining <= 0.0) {
        if (!attach_ball()) {
          posture = SQUATTED;
          printf("Pickup aborted: align basketball near hands first.\n");
        } else if (grip_only_test) {
          posture = SQUATTED;
          printf("GRIP TEST COMPLETE: fingers held at %.2f. Inspect before pressing L.\n", HAND_CARRY_ANGLE);
        } else if (start_motion(stand_from_squat)) {
          posture = STANDING_UP;
          printf("GRIP REACHED %.2f; starting stand motion now.\n", HAND_CARRY_ANGLE);
        } else {
          posture = SQUATTED;
          fprintf(stderr, "Stand motion missing; remaining squatted.\n");
        }
      }
    }
    return;
  }
  if (posture == SQUATTING && squat && wbu_motion_is_over(squat)) {
    posture = SQUATTED;
    currently_playing = NULL;
    printf("Squat completed. Press L to stand.\n");
  } else if (posture == STANDING_UP && stand_from_squat && wbu_motion_is_over(stand_from_squat)) {
    posture = STANDING;
    currently_playing = NULL;
    printf("Stand-up motion completed.\n");
  }
}

static void handle_key(int key) {
  // Hand controls remain available while squatting.
  if (key == WB_KEYBOARD_UP) {
    hand_target = HAND_OPEN_ANGLE;
    if (ball_attached) release_ball(0);
    if (posture == PREPARING_GRIP) {
      posture = SQUATTED;
      printf("Auto-grip canceled. Still squatted.\n");
    }
    printf("Opening fingers gradually...\n");
    return;
  }
  if (key == WB_KEYBOARD_DOWN) {
    hand_target = HAND_CLOSED_ANGLE;
    if (posture == PREPARING_GRIP) {
      posture = SQUATTED;
      printf("Auto-grip canceled. Still squatted.\n");
    }
    printf("Closing fingers gradually...\n");
    return;
  }

  if (key == 'K' || key == 'k') {
    if (posture == STANDING &&
        (!currently_playing || wbu_motion_is_over(currently_playing)) &&
        start_motion(squat)) {
      posture = SQUATTING;
      printf("Squatting...\n");
    }
    return;
  }
  if (key == 'G' || key == 'g') {
    if (posture == SQUATTED && !ball_attached) {
      grip_only_test = 1;
      hand_target = HAND_CARRY_ANGLE;
      grip_settle_remaining = HAND_GRIP_SETTLE_SECONDS;
      posture = PREPARING_GRIP;
      printf("G TEST: fingers closing to %.2f; NAO stays squatted.\n", HAND_CARRY_ANGLE);
    }
    return;
  }
  if (key == 'L' || key == 'l') {
    if (posture == SQUATTED && stand_from_squat) {
      if (ball_attached) {
        if (start_motion(stand_from_squat)) {
          posture = STANDING_UP;
          printf("L: standing while carrying assisted basketball.\n");
        }
      } else {
        grip_only_test = 0;
        hand_target = HAND_CARRY_ANGLE;
        grip_settle_remaining = HAND_GRIP_SETTLE_SECONDS;
        posture = PREPARING_GRIP;
        printf("L: partial hand close, attach ball if near hands, then stand.\n");
      }
    } else {
      printf("L ignored: complete the squat first.\n");
    }
    return;
  }

  if (key == 'H' || key == 'h') {
    release_ball(0);  /* Gentle drop / test release. */
    return;
  }
  if (key == 'R' || key == 'r') {
    release_ball(1);  /* Demonstration release, not a trained shot. */
    return;
  }

  // Don't interrupt a squat or walk in a squatted posture.
  if (posture != STANDING)
    return;

  switch (key) {
    case 'W':
    case 'w':
      start_motion(forwards);
      break;
    case 'S':
    case 's':
      start_motion(ball_attached ? backwards_carry : backwards);
      break;
    case 'A':
    case 'a':
      start_motion(ball_attached ? side_step_left_carry : side_step_left);
      break;
    case 'D':
    case 'd':
      start_motion(ball_attached ? side_step_right_carry : side_step_right);
      break;
    case 'O':
    case 'o':
      start_motion(shoot);
      break;
    case WB_KEYBOARD_LEFT:
      start_motion(turn_left);
      break;
    case WB_KEYBOARD_RIGHT:
      start_motion(turn_right);
      break;
    default:
      break;
  }
}

int main(void) {
  wb_robot_init();
  const int time_step = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(time_step);
  initialize_hands();
  /* Start from open fingers; press DOWN to close slowly. */
  set_hands_angle(HAND_OPEN_ANGLE);
  printf("Grip target: %.2f (open %.2f, full fist %.2f).\n", HAND_CARRY_ANGLE, HAND_OPEN_ANGLE, HAND_CLOSED_ANGLE);
  load_motion_files();
  initialize_supervisor_grasp();
  if (robot_node && ball_node && ball_translation) {
    if (locate_carry_visual()) {
      wb_supervisor_field_set_sf_vec3f(carry_visual_translation, VISUAL_BALL_PARK);
      printf("Static carry visual ready: no dynamic node creation or deletion.\n");
    }
  }

  printf("NAO: W/S walk, A/D sidestep, LEFT/RIGHT turn, UP open, DOWN fist, K squat, G assisted grasp, L grasp+stand, H drop, R release test.\n");

  while (wb_robot_step(time_step) != -1) {
    update_hands(time_step);
    update_posture(time_step);
    follow_attached_ball();
    const int key = wb_keyboard_get_key();
    if (key != -1)
      handle_key(key);
  }

  wb_robot_cleanup();
  return 0;
}
