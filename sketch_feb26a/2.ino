// #include <micro_ros_arduino.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <geometry_msgs/msg/twist.h>
// #include <nav_msgs/msg/odometry.h>
// #include <geometry_msgs/msg/quaternion.h>
// #include <geometry_msgs/msg/transform_stamped.h>
// #include <tf2_msgs/msg/tf_message.h>

// // ROS 2 nodes and topics
// rcl_subscription_t twist_subscriber;
// geometry_msgs__msg__Twist twist_msg;

// rcl_publisher_t odom_publisher;
// nav_msgs__msg__Odometry odom_msg;

// rcl_publisher_t tf_publisher;
// tf2_msgs__msg__TFMessage tf_msg;
// geometry_msgs__msg__TransformStamped transform_stamped;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// // Encoder pins
// #define LEFT_A 35
// #define LEFT_B 34
// #define RIGHT_A 36  // EC2
// #define RIGHT_B 39  // EC1

// // Encoder variables
// volatile long leftEncoderCount = 0;
// volatile long rightEncoderCount = 0;
// volatile long lastLeftCount = 0;
// volatile long lastRightCount = 0;

// // Motor control pins
// int INA1 = 25;  int INB1 = 26;
// int INA2 = 33;  int INB2 = 32;


// // Robot odometry state
// float robotX = 0.0;
// float robotY = 0.0;
// float robotTheta = 0.0;

// // Robot constants
// const float WHEEL_DIAMETER = 12.1;        // cm
// const float WHEEL_BASE = 63.85;           // cm
// const float COUNTS_PER_REVOLUTION = 54000;
// const float CM_PER_COUNT = 0.002274;      // Calibrated
// const float CM_TO_M = 0.01;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); } }
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

// void error_loop() {
//   pinMode(2, OUTPUT);
//   while (1) {
//     digitalWrite(2, HIGH);
//     delay(100);
//     digitalWrite(2, LOW);
//     delay(100);
//   }
// }

// // yaw → quaternion
// geometry_msgs__msg__Quaternion yaw_to_quaternion(float yaw) {
//   geometry_msgs__msg__Quaternion q;
//   q.x = 0.0;
//   q.y = 0.0;
//   q.z = sin(yaw / 2.0);
//   q.w = cos(yaw / 2.0);
//   return q;
// }
// void move(int rightspeed, int leftspeed) {
//   leftspeed  = constrain(leftspeed,  -100, 100);
//   rightspeed = constrain(rightspeed, -100, 100);

//   int leftPWM  = map(abs(leftspeed),  0, 100, 0, 255);
//   int rightPWM = map(abs(rightspeed), 0, 100, 0, 255);

//   // LEFT MOTOR
//   if (leftspeed > 0) {
//     // forward
//     analogWrite(INA1, leftPWM);
//     analogWrite(INB1, 0);
//   } else if (leftspeed < 0) {
//     // backward
//     analogWrite(INA1, 0);
//     analogWrite(INB1, leftPWM);
//   } else {
//     analogWrite(INA1, 0);
//     analogWrite(INB1, 0);
//   }

//   // RIGHT MOTOR
//   if (rightspeed > 0) {
//     // forward
//     analogWrite(INA2, rightPWM);
//     analogWrite(INB2, 0);
//   } else if (rightspeed < 0) {
//     // backward
//     analogWrite(INA2, 0);
//     analogWrite(INB2, rightPWM);
//   } else {
//     analogWrite(INA2, 0);
//     analogWrite(INB2, 0);
//   }
// }


// // Encoder setup
// void setupEncoders() {
//   pinMode(LEFT_A, INPUT);
//   pinMode(LEFT_B, INPUT);
//   pinMode(RIGHT_A, INPUT);
//   pinMode(RIGHT_B, INPUT);
  
//   attachInterrupt(digitalPinToInterrupt(LEFT_A), leftEncoderISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);
// }

// void leftEncoderISR() {
//   if (digitalRead(LEFT_A) == digitalRead(LEFT_B)) leftEncoderCount--;
//   else leftEncoderCount++;
// }

// void rightEncoderISR() {
//   if (digitalRead(RIGHT_A) == digitalRead(RIGHT_B)) rightEncoderCount++;
//   else rightEncoderCount--;
// }

// // Update odometry
// void updateOdometry() {
//   long leftDelta = leftEncoderCount - lastLeftCount;
//   long rightDelta = rightEncoderCount - lastRightCount;
  
//   float leftDistance = leftDelta * CM_PER_COUNT;
//   float rightDistance = rightDelta * CM_PER_COUNT;
  
//   float deltaDistance = (leftDistance + rightDistance) / 2.0;
//   float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE;
  
//   robotX += deltaDistance * cos(robotTheta + deltaTheta / 2.0);
//   robotY += deltaDistance * sin(robotTheta + deltaTheta / 2.0);
//   robotTheta += deltaTheta;
  
//   while (robotTheta > PI) robotTheta -= 2 * PI;
//   while (robotTheta < -PI) robotTheta += 2 * PI;
  
//   lastLeftCount = leftEncoderCount;
//   lastRightCount = rightEncoderCount;
// }

// // cmd_vel callback
// void cmd_vel_callback(const void *msgin) {
//   const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
//   float linear = msg->linear.x;
//   float angular = msg->angular.z;

//   int leftspeed = (int)((linear - angular) * 100);
//   int rightspeed = (int)((linear + angular) * 100);

//   move(rightspeed, leftspeed);
// }

// // Timer callback → odom + tf
// void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     updateOdometry();

//     // --- Odometry message ---
//     odom_msg.header.stamp.sec = millis() / 1000;
//     odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
//     odom_msg.header.frame_id.data = (char*)"odom";
//     odom_msg.child_frame_id.data = (char*)"base_link";

//     odom_msg.pose.pose.position.x = robotX * CM_TO_M;
//     odom_msg.pose.pose.position.y = robotY * CM_TO_M;
//     odom_msg.pose.pose.orientation = yaw_to_quaternion(robotTheta);

//     // Velocity
//     static unsigned long last_time = 0;
//     unsigned long current_time = millis();
//     float dt = (current_time - last_time) / 1000.0;
//     if (dt > 0) {
//       static long prev_left = 0, prev_right = 0;
//       long delta_left = leftEncoderCount - prev_left;
//       long delta_right = rightEncoderCount - prev_right;

//       float left_dist = delta_left * CM_PER_COUNT * CM_TO_M;
//       float right_dist = delta_right * CM_PER_COUNT * CM_TO_M;

//       float v = (left_dist + right_dist) / (2.0 * dt);
//       float w = (right_dist - left_dist) / ((WHEEL_BASE * CM_TO_M) * dt);

//       odom_msg.twist.twist.linear.x = v;
//       odom_msg.twist.twist.angular.z = w;

//       prev_left = leftEncoderCount;
//       prev_right = rightEncoderCount;
//     }
//     last_time = current_time;

//     // Covariance
//     for (int i = 0; i < 36; i++) {
//       odom_msg.pose.covariance[i] = 0.0;
//       odom_msg.twist.covariance[i] = 0.0;
//     }
//     odom_msg.pose.covariance[0] = 0.01;
//     odom_msg.pose.covariance[7] = 0.01;
//     odom_msg.pose.covariance[35] = 0.1;
//     odom_msg.twist.covariance[0] = 0.01;
//     odom_msg.twist.covariance[35] = 0.1;

//     RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

//     // --- TF broadcaster ---
//     transform_stamped.header = odom_msg.header;
//     transform_stamped.child_frame_id.data = (char*)"base_link";
//     transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
//     transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
//     transform_stamped.transform.translation.z = 0.0;
//     transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;

//     tf_msg.transforms.size = 1;
//     tf_msg.transforms.data = &transform_stamped;

//     RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));

//     // Debug
//     Serial.print("X: "); Serial.print(robotX, 2);
//     Serial.print(" Y: "); Serial.print(robotY, 2);
//     Serial.print(" Theta: "); Serial.println(robotTheta * 180.0 / PI, 2);
//   }
// }

// void setup() {
//   Serial.begin(115200);
  
//   set_microros_transports();
//   setupEncoders();

//   pinMode(INA1, OUTPUT); pinMode(INB1, OUTPUT);
//   pinMode(INA2, OUTPUT); pinMode(INB2, OUTPUT);


//   allocator = rcl_get_default_allocator();
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//   RCCHECK(rclc_node_init_default(&node, "RMRC_esp32", "", &support));

//   RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node,
//             ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
//   RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
//             ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
//   RCCHECK(rclc_publisher_init_default(&tf_publisher, &node,
//             ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf"));

//   const unsigned int timer_timeout = 100;
//   RCCHECK(rclc_timer_init_default(&timer, &support,
//             RCL_MS_TO_NS(timer_timeout), timer_callback));

//   RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));
//   RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber,
//             &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

//   Serial.println("Robot initialized: /cmd_vel subscriber, /odom + /tf publisher");
// }

// void loop() {
//   delay(100);
//   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }