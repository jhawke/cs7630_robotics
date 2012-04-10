
(cl:in-package :asdf)

(defsystem "raptor_commander-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getAdviceFLEE" :depends-on ("_package_getAdviceFLEE"))
    (:file "_package_getAdviceFLEE" :depends-on ("_package"))
    (:file "getAdviceDET" :depends-on ("_package_getAdviceDET"))
    (:file "_package_getAdviceDET" :depends-on ("_package"))
    (:file "getAdviceFD" :depends-on ("_package_getAdviceFD"))
    (:file "_package_getAdviceFD" :depends-on ("_package"))
  ))