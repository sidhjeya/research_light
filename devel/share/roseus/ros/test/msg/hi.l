;; Auto-generated. Do not edit!


(when (boundp 'test::hi)
  (if (not (find-package "TEST"))
    (make-package "TEST"))
  (shadow 'hi (find-package "TEST")))
(unless (find-package "TEST::HI")
  (make-package "TEST::HI"))

(in-package "ROS")
;;//! \htmlinclude hi.msg.html


(defclass test::hi
  :super ros::object
  :slots (_real _imaginary ))

(defmethod test::hi
  (:init
   (&key
    ((:real __real) 0.0)
    ((:imaginary __imaginary) 0.0)
    )
   (send-super :init)
   (setq _real (float __real))
   (setq _imaginary (float __imaginary))
   self)
  (:real
   (&optional __real)
   (if __real (setq _real __real)) _real)
  (:imaginary
   (&optional __imaginary)
   (if __imaginary (setq _imaginary __imaginary)) _imaginary)
  (:serialization-length
   ()
   (+
    ;; float32 _real
    4
    ;; float32 _imaginary
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _real
       (sys::poke _real (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imaginary
       (sys::poke _imaginary (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _real
     (setq _real (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imaginary
     (setq _imaginary (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get test::hi :md5sum-) "54da470dccf15d60bd273ab751e1c0a1")
(setf (get test::hi :datatype-) "test/hi")
(setf (get test::hi :definition-)
      "float32 real
float32 imaginary
")



(provide :test/hi "54da470dccf15d60bd273ab751e1c0a1")


