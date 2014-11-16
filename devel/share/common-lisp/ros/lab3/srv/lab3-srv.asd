
(cl:in-package :asdf)

(defsystem "lab3-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "astar" :depends-on ("_package_astar"))
    (:file "_package_astar" :depends-on ("_package"))
  ))