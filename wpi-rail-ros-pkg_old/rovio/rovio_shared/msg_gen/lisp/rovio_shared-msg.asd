
(cl:in-package :asdf)

(defsystem "rovio_shared-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "man_drv" :depends-on ("_package_man_drv"))
    (:file "_package_man_drv" :depends-on ("_package"))
  ))