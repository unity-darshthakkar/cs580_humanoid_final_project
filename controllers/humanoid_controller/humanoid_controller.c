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
#include <webots/utils/motion.h>

#define PHALANX_MAX 8

static WbDeviceTag left_phalanx[PHALANX_MAX];
static WbDeviceTag right_phalanx[PHALANX_MAX];

static WbMotionRef forwards = NULL;
static WbMotionRef backwards = NULL;
static WbMotionRef side_step_left = NULL;
static WbMotionRef side_step_right = NULL;
static WbMotionRef turn_left = NULL;
static WbMotionRef turn_right = NULL;
static WbMotionRef squat = NULL;
static WbMotionRef stand_from_squat = NULL;
static WbMotionRef currently_playing = NULL;

typedef enum {
  STANDING,
  SQUATTING,
  SQUATTED,
  STANDING_UP
} PostureState;

static PostureState posture = STANDING;

static void load_motion_files(void) {
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  turn_left = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right = wbu_motion_new("../../motions/TurnRight60.motion");
  squat = wbu_motion_new("../../motions/Squat.motion");
  stand_from_squat = wbu_motion_new("../../motions/StandFromSquat.motion");
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

static void update_posture(void) {
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
    set_hands_angle(0.96);
    return;
  }
  if (key == WB_KEYBOARD_DOWN) {
    set_hands_angle(0.0);
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
  if (key == 'L' || key == 'l') {
    if (posture == SQUATTED && start_motion(stand_from_squat)) {
      posture = STANDING_UP;
      printf("Standings up...\n");
    }
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
      start_motion(backwards);
      break;
    case 'A':
    case 'a':
      start_motion(side_step_left);
      break;
    case 'D':
    case 'd':
      start_motion(side_step_right);
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
  load_motion_files();

  printf("NAO: W/S walk, A/D sidestep, LEFT/RIGHT turn, UP/DOWN hands, K squat, L stand.\n");

  while (wb_robot_step(time_step) != -1) {
    update_posture();
    const int key = wb_keyboard_get_key();
    if (key != -1)
      handle_key(key);
  }

  wb_robot_cleanup();
  return 0;
}
