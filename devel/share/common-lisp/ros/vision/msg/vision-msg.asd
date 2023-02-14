
(cl:in-package :asdf)

(defsystem "vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "custMsg" :depends-on ("_package_custMsg"))
    (:file "_package_custMsg" :depends-on ("_package"))
  ))