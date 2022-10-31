# TF

* #### 오일러 각도('Euler angle')

x, y, z 축을 기준으로 회전시키는 흔히 알고있는 각도계를 의미한다.

**짐벌락('Gimbal-lock')**이라는 문제가 있기 때문에 모든 각도 변환을 표현하는데 한계가 있다.

짐벌락이란 두 회전 축이 겹치는 현상이다.



* #### 쿼터니언('Quaternion')

**4개의 성분(x, y, z, w)**으로 이루어져 있고 해당 성분은 벡터(x, y, z)와 스칼라(w)를 의미한다.

쿼터니언은 방향(orientation)과 회전(rotation) 둘을 다 표현할 수 있다.

하지만 쿼터니언의 회전은 한 orientation 에서 다른 orientation 으로 측정하기에 180 보다 큰 값을 표현할 수 없다는 단점이 있다.  (?)

이 점이 쿼터니언을 직관적으로 이해할 수 없는 큰 이유 중 하나이다.



* #### 사용되면 함수

  * ##### Broadcaster 생성

    br = tf.TransformBroadcaster()

    + ###### sendTransform() 함수

      br.sendTransform( translation, quaternion, 현재시간, 생성할 tf 이름, 부모 tf 이름 )

      rviz에서 생성된 tf를 볼 수 있다.

  * ##### Listener 생성

    listener = tf.TransformListener()

    + ###### lookupTransform() 함수

      (trans, rot) = listener.lookupTransform( 자식 tf 이름, 부모 tf 이름, 현재시간 )

      **trans** 는 부모에 비해 자식 프레임의 선형 변환(x, y, z) 선형 변환이고

      **rot** 은 부모 방향에서 자식 방향으로 회전하는 데 필요한 (x, y, z, w) 쿼터니온이다.

      일반적으로 while 문 안에서 try 안에 쓰여진다.

  * ##### transformations

    + ###### quaternion_from_euler() 함수

      roll, pitch, yaw = tf.transformations.euler_from_quaternion( x, y, z, w )

      rad 단위로 반환한다.

    + ###### euler_from_quaternion() 함수

      x, y, z, w = tf.transformations.quaternion_from_euler( roll, pitch, yaw )

  * ##### tf_pub.py

  ```c++
  #!/usr/bin/env python
  # -*- coding: utf-8 -*-
  
  import rospy
  import tf
  from std_msgs.msg import Bool
  from geometry_msgs.msg import Pose
  
  tf_start_check = False
  
  #control에서 aruco 발견 여부 확인
  def aruco_tf_callback(check):
      global tf_start_check
      tf_start_check = check.data
  
  
  def main():
      rospy.init_node("tf_pub")
      global tf_start_check
      pose = Pose()
      rate = rospy.Rate(10)
      listener = tf.TransformListener()
      a_about_m_pub = rospy.Publisher('a_about_m_pos', Pose, queue_size=10)
      aruco_tf_check_pub = rospy.Publisher('aruco_tf_check', Bool, queue_size=10)
      while not rospy.is_shutdown():
          rospy.Subscriber('aruco_tf_start', Bool, aruco_tf_callback)
          if tf_start_check:
              try:
                  (trans, rot) = listener.lookupTransform('/base_link', 'aruco_pose', rospy.Time(0))
                  rospy.loginfo("tf_success")
              except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  rospy.loginfo("tf_except")
                  continue
              pose.position.x = trans[0] + 0.02
              pose.position.y = trans[1]
              pose.position.z = trans[2]
              pose.orientation.x = rot[0]
              pose.orientation.y = rot[1]
              pose.orientation.z = rot[2]
              pose.orientation.w = rot[3]
              rospy.loginfo("{}".format(pose))
              rospy.loginfo("aruco tf_pub end :)")
              a_about_m_pub.publish(pose)
              aruco_tf_check_pub.publish(True)
          else:
              aruco_tf_check_pub.publish(False)
          rate.sleep()
  
  
  if __name__ == "__main__":
      try:
          main()
      except rospy.ROSInterruptException:
          pass
  ```

  + ##### tf_sub.py

  ```c++
  #!/usr/bin/env python
  # -*- coding: utf-8 -*-
  
  import rospy
  import tf
  import numpy as np
  from geometry_msgs.msg import Pose
  
  #mani_pose에 따른 캠에서 로봇으로 tf
  def draw_cam(mani_pose):
      rospy.loginfo("tf_sub_draw_cam")
      br = tf.TransformBroadcaster()
  
      br.sendTransform((mani_pose.position.x, mani_pose.position.y, mani_pose.position.z),
                  (0, 0, 0, 1),
                  rospy.Time.now(),
                  "mani_pose",
                  "/base_link")
  
      br.sendTransform((-0.06, 0, 0.04), (0, 0, 0, 1), rospy.Time.now(), "cam_test", "mani_pose")
  
  #카메라 축에서 로봇 축으로 tf
  def draw_rgb():
      rospy.loginfo("tf_sub_draw_rgb")
      br = tf.TransformBroadcaster()
      
      x, y, z, w = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)
  
      br.sendTransform((0, 0, 0),
                      (x, y, z, w),
                      rospy.Time.now(),
                      "rgb_test",
                      "cam_test")
  
  #aruco에서 카메라로 tf
  def draw_aruco(aruco_pos):
      rospy.loginfo("tf_sub_draw_aruco")
      br = tf.TransformBroadcaster()
  
      draw_rgb()
  
      x,y,z,w = tf.transformations.quaternion_from_euler(aruco_pos.orientation.x, aruco_pos.orientation.y, aruco_pos.orientation.z)
  
      br.sendTransform((aruco_pos.position.x, aruco_pos.position.y, aruco_pos.position.z),
                      (x, y, z, w),
                      rospy.Time.now(),
                      "aruco_pose",
                      "rgb_test")
  
  
  def main():
      rospy.init_node("tf_sub")
      rospy.Subscriber('mani_pos', Pose, draw_cam)
      rospy.Subscriber('aruco_xyzw', Pose, draw_aruco)
      rospy.spin()
  
  
  if __name__ == "__main__":
      try:
          main()
      except rospy.ROSInterruptException:
          pass
  ```

  











