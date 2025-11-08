
(cl:in-package :asdf)

(defsystem "ros_volume_estimation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ToggleProcessing" :depends-on ("_package_ToggleProcessing"))
    (:file "_package_ToggleProcessing" :depends-on ("_package"))
  ))