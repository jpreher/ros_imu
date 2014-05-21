
(cl:in-package :asdf)

(defsystem "tau_under-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "tau_under_msg" :depends-on ("_package_tau_under_msg"))
    (:file "_package_tau_under_msg" :depends-on ("_package"))
  ))