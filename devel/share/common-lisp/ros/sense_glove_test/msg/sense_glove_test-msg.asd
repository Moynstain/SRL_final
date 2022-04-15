
(cl:in-package :asdf)

(defsystem "sense_glove_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "glove" :depends-on ("_package_glove"))
    (:file "_package_glove" :depends-on ("_package"))
  ))