;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_manipulation::StopController)
  (if (not (find-package "DYNAMIXEL_MANIPULATION"))
    (make-package "DYNAMIXEL_MANIPULATION"))
  (shadow 'StopController (find-package "DYNAMIXEL_MANIPULATION")))
(unless (find-package "DYNAMIXEL_MANIPULATION::STOPCONTROLLER")
  (make-package "DYNAMIXEL_MANIPULATION::STOPCONTROLLER"))
(unless (find-package "DYNAMIXEL_MANIPULATION::STOPCONTROLLERREQUEST")
  (make-package "DYNAMIXEL_MANIPULATION::STOPCONTROLLERREQUEST"))
(unless (find-package "DYNAMIXEL_MANIPULATION::STOPCONTROLLERRESPONSE")
  (make-package "DYNAMIXEL_MANIPULATION::STOPCONTROLLERRESPONSE"))

(in-package "ROS")





(defclass dynamixel_manipulation::StopControllerRequest
  :super ros::object
  :slots (_controller_name ))

(defmethod dynamixel_manipulation::StopControllerRequest
  (:init
   (&key
    ((:controller_name __controller_name) "")
    )
   (send-super :init)
   (setq _controller_name (string __controller_name))
   self)
  (:controller_name
   (&optional __controller_name)
   (if __controller_name (setq _controller_name __controller_name)) _controller_name)
  (:serialization-length
   ()
   (+
    ;; string _controller_name
    4 (length _controller_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _controller_name
       (write-long (length _controller_name) s) (princ _controller_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _controller_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _controller_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dynamixel_manipulation::StopControllerResponse
  :super ros::object
  :slots (_success _reason ))

(defmethod dynamixel_manipulation::StopControllerResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _reason (string __reason))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dynamixel_manipulation::StopController
  :super ros::object
  :slots ())

(setf (get dynamixel_manipulation::StopController :md5sum-) "b3f1f861a8aef49071034d2f06308f6e")
(setf (get dynamixel_manipulation::StopController :datatype-) "dynamixel_manipulation/StopController")
(setf (get dynamixel_manipulation::StopController :request) dynamixel_manipulation::StopControllerRequest)
(setf (get dynamixel_manipulation::StopController :response) dynamixel_manipulation::StopControllerResponse)

(defmethod dynamixel_manipulation::StopControllerRequest
  (:response () (instance dynamixel_manipulation::StopControllerResponse :init)))

(setf (get dynamixel_manipulation::StopControllerRequest :md5sum-) "b3f1f861a8aef49071034d2f06308f6e")
(setf (get dynamixel_manipulation::StopControllerRequest :datatype-) "dynamixel_manipulation/StopControllerRequest")
(setf (get dynamixel_manipulation::StopControllerRequest :definition-)
      "string controller_name
---
bool success
string reason
")

(setf (get dynamixel_manipulation::StopControllerResponse :md5sum-) "b3f1f861a8aef49071034d2f06308f6e")
(setf (get dynamixel_manipulation::StopControllerResponse :datatype-) "dynamixel_manipulation/StopControllerResponse")
(setf (get dynamixel_manipulation::StopControllerResponse :definition-)
      "string controller_name
---
bool success
string reason
")



(provide :dynamixel_manipulation/StopController "b3f1f861a8aef49071034d2f06308f6e")


