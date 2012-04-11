
(cl:in-package :asdf)

(defsystem "raptor_commander-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getAdvicePD" :depends-on ("_package_getAdvicePD"))
    (:file "_package_getAdvicePD" :depends-on ("_package"))
    (:file "getAdviceFD" :depends-on ("_package_getAdviceFD"))
    (:file "_package_getAdviceFD" :depends-on ("_package"))
  ))