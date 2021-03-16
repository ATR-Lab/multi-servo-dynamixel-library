;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_manipulation::SetSpeed)
  (if (not (find-package "DYNAMIXEL_MANIPULATION"))
    (make-package "DYNAMIXEL_MANIPULATION"))
  (shadow 'SetSpeed (find-package "DYNAMIXEL_MANIPULATION")))
(unless (find-package "DYNAMIXEL_MANIPULATION::SETSPEED")
  (make-package "DYNAMIXEL_MANIPULATION::SETSPEED"))
(unless (find-package "DYNAMIXEL_MANIPULATION::SETSPEEDREQUEST")
  (make-package "DYNAMIXEL_MANIPULATION::SETSPEEDREQUEST"))
(unless (find-package "DYNAMIXEL_MANIPULATION::SETSPEEDRESPONSE")
  (make-package "DYNAMIXEL_MANIPULATION::SETSPEEDRESPONSE"))

(in-package "ROS")





(defclass dynamixel_manipulation::SetSpeedRequest
  :super ros::object
  :slots (_speed ))

(defmethod dynamixel_manipulation::SetSpeedRequest
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float64 _speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dynamixel_manipulation::SetSpeedResponse
  :super ros::object
  :slots ())

(defmethod dynamixel_manipulation::SetSpeedResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dynamixel_manipulation::SetSpeed
  :super ros::object
  :slots ())

(setf (get dynamixel_manipulation::SetSpeed :md5sum-) "4641bb0523a3557209606d9bd91ce33a")
(setf (get dynamixel_manipulation::SetSpeed :datatype-) "dynamixel_manipulation/SetSpeed")
(setf (get dynamixel_manipulation::SetSpeed :request) dynamixel_manipulation::SetSpeedRequest)
(setf (get dynamixel_manipulation::SetSpeed :response) dynamixel_manipulation::SetSpeedResponse)

(defmethod dynamixel_manipulation::SetSpeedRequest
  (:response () (instance dynamixel_manipulation::SetSpeedResponse :init)))

(setf (get dynamixel_manipulation::SetSpeedRequest :md5sum-) "4641bb0523a3557209606d9bd91ce33a")
(setf (get dynamixel_manipulation::SetSpeedRequest :datatype-) "dynamixel_manipulation/SetSpeedRequest")
(setf (get dynamixel_manipulation::SetSpeedRequest :definition-)
      "float64 speed
---

")

(setf (get dynamixel_manipulation::SetSpeedResponse :md5sum-) "4641bb0523a3557209606d9bd91ce33a")
(setf (get dynamixel_manipulation::SetSpeedResponse :datatype-) "dynamixel_manipulation/SetSpeedResponse")
(setf (get dynamixel_manipulation::SetSpeedResponse :definition-)
      "float64 speed
---

")



(provide :dynamixel_manipulation/SetSpeed "4641bb0523a3557209606d9bd91ce33a")


