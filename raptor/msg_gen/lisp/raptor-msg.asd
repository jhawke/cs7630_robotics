
(cl:in-package :asdf)

(defsystem "raptor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "abs_pos_req" :depends-on ("_package_abs_pos_req"))
    (:file "_package_abs_pos_req" :depends-on ("_package"))
    (:file "rel_pos_req" :depends-on ("_package_rel_pos_req"))
    (:file "_package_rel_pos_req" :depends-on ("_package"))
    (:file "Vector3" :depends-on ("_package_Vector3"))
    (:file "_package_Vector3" :depends-on ("_package"))
  ))