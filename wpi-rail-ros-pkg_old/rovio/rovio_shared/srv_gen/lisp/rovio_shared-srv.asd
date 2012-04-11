
(cl:in-package :asdf)

(defsystem "rovio_shared-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "rovio_position" :depends-on ("_package_rovio_position"))
    (:file "_package_rovio_position" :depends-on ("_package"))
    (:file "wav_play" :depends-on ("_package_wav_play"))
    (:file "_package_wav_play" :depends-on ("_package"))
    (:file "head_ctrl" :depends-on ("_package_head_ctrl"))
    (:file "_package_head_ctrl" :depends-on ("_package"))
    (:file "twist_srv" :depends-on ("_package_twist_srv"))
    (:file "_package_twist_srv" :depends-on ("_package"))
    (:file "man_drv_srv" :depends-on ("_package_man_drv_srv"))
    (:file "_package_man_drv_srv" :depends-on ("_package"))
  ))