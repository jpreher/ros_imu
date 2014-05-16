
(cl:in-package :asdf)

(defsystem "leg_pose-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "legPose" :depends-on ("_package_legPose"))
    (:file "_package_legPose" :depends-on ("_package"))
  ))