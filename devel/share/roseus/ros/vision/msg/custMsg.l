;; Auto-generated. Do not edit!


(when (boundp 'vision::custMsg)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'custMsg (find-package "VISION")))
(unless (find-package "VISION::CUSTMSG")
  (make-package "VISION::CUSTMSG"))

(in-package "ROS")
;;//! \htmlinclude custMsg.msg.html


(defclass vision::custMsg
  :super ros::object
  :slots (_index _x _y _z ))

(defmethod vision::custMsg
  (:init
   (&key
    ((:index __index) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; int32 _index
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _index
       (write-long _index s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vision::custMsg :md5sum-) "2622e279f148bab2f2656b233b9ab6cd")
(setf (get vision::custMsg :datatype-) "vision/custMsg")
(setf (get vision::custMsg :definition-)
      "int32 index
float32 x
float32 y
float32 z
")



(provide :vision/custMsg "2622e279f148bab2f2656b233b9ab6cd")


