cmake_minimum_required(VERSION 3.5)
project(roller_eye)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  alexskill.msg
  contour.msg
  detect.msg
  frame.msg
  patrol_status.msg
  point.msg
  record.msg
  status.msg
  task.msg
)

add_service_files(
  FILES
  adjust_ligth.srv
  ai_get_detect_setting.srv
  ai_set_detect_setting.srv
  algo_action.srv
  algo_move.srv
  algo_roll.srv
  enable_vio.srv
  getOdom.srv
  led_all_on.srv
  nav_calibration_get_status.srv
  nav_cancel.srv
  nav_delete_path.srv
  nav_exit.srv
  nav_get_patrol_name.srv
  nav_get_status.srv
  nav_list_path.srv
  nav_mag_calibra.srv
  nav_path_save.srv
  nav_path_start.srv
  nav_patrol.srv
  nav_patrol_stop.srv
  nav_waypoint_add.srv
  nav_waypoint_query.srv
  night_get.srv
  sched_add_mod.srv
  sched_delete.srv
  sched_delete_byname.srv
  sched_list.srv
  stop_detect.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
