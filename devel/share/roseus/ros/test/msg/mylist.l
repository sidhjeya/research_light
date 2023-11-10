;; Auto-generated. Do not edit!


(when (boundp 'test::mylist)
  (if (not (find-package "TEST"))
    (make-package "TEST"))
  (shadow 'mylist (find-package "TEST")))
(unless (find-package "TEST::MYLIST")
  (make-package "TEST::MYLIST"))

(in-package "ROS")
;;//! \htmlinclude mylist.msg.html


(defclass test::mylist
  :super ros::object
  :slots (_data ))

(defmethod test::mylist
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get test::mylist :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get test::mylist :datatype-) "test/mylist")
(setf (get test::mylist :definition-)
      "float32 data
")



(provide :test/mylist "73fcbf46b49191e672908e50842a83d4")


