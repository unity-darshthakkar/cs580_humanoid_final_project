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

/* CS580 Humanoid Sorting: keyboard navigation and hand control only. */
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
static WbMotionRef currently_playing = NULL;

static void load_motion_files(void) {
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  turn_left = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right = wbu_motion_new("../../motions/TurnRight60.motion");
}

static void start_motion(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Failed to load motion; check ../../motions/\n");
    return;
  }
  if (currently_playing)
    wbu_motion_stop(currently_playing);
  wbu_motion_play(motion);
  currently_playing = motion;
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
    WbDeviceTag motors[2] = {left_phalanx[i], right_phalanx[i]};
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

static void handle_key(int key) {
  // Webots returns uppercase ASCII for ordinary letter keys.
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
    case WB_KEYBOARD_UP:
      set_hands_angle(0.96);  // Open hands
      break;
    case WB_KEYBOARD_DOWN:
      set_hands_angle(0.0);   // Close hands
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

  printf("NAO controller: W/S walk, A/D sidestep, LEFT/RIGHT turn, UP open hands, DOWN close hands.\n");

  while (wb_robot_step(time_step) != -1) {
    const int key = wb_keyboard_get_key();
    if (key != -1)
      handle_key(key);
  }

  wb_robot_cleanup();
  return 0;
}
