
(cl:in-package :asdf)

(defsystem "raptor-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "obstacle_histogram" :depends-on ("_package_obstacle_histogram"))
    (:file "_package_obstacle_histogram" :depends-on ("_package"))
    (:file "polar_histogram" :depends-on ("_package_polar_histogram"))
    (:file "_package_polar_histogram" :depends-on ("_package"))
  ))