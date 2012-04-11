
(cl:in-package :asdf)

(defsystem "raptor-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :raptor-msg
)
  :components ((:file "_package")
    (:file "obstacle_histogram" :depends-on ("_package_obstacle_histogram"))
    (:file "_package_obstacle_histogram" :depends-on ("_package"))
    (:file "polar_histogram" :depends-on ("_package_polar_histogram"))
    (:file "_package_polar_histogram" :depends-on ("_package"))
    (:file "distance_adv_srv" :depends-on ("_package_distance_adv_srv"))
    (:file "_package_distance_adv_srv" :depends-on ("_package"))
    (:file "light_level_srv" :depends-on ("_package_light_level_srv"))
    (:file "_package_light_level_srv" :depends-on ("_package"))
    (:file "twist_srv" :depends-on ("_package_twist_srv"))
    (:file "_package_twist_srv" :depends-on ("_package"))
    (:file "state_set_srv" :depends-on ("_package_state_set_srv"))
    (:file "_package_state_set_srv" :depends-on ("_package"))
  ))