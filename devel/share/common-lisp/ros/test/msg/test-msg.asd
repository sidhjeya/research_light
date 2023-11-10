
(cl:in-package :asdf)

(defsystem "test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "hi" :depends-on ("_package_hi"))
    (:file "_package_hi" :depends-on ("_package"))
    (:file "mylist" :depends-on ("_package_mylist"))
    (:file "_package_mylist" :depends-on ("_package"))
  ))