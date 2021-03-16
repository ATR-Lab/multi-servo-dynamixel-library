
(cl:in-package :asdf)

(defsystem "dynamixel_manipulation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RestartController" :depends-on ("_package_RestartController"))
    (:file "_package_RestartController" :depends-on ("_package"))
    (:file "SetSpeed" :depends-on ("_package_SetSpeed"))
    (:file "_package_SetSpeed" :depends-on ("_package"))
    (:file "StartController" :depends-on ("_package_StartController"))
    (:file "_package_StartController" :depends-on ("_package"))
    (:file "StopController" :depends-on ("_package_StopController"))
    (:file "_package_StopController" :depends-on ("_package"))
  ))