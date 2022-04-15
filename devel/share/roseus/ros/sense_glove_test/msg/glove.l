;; Auto-generated. Do not edit!


(when (boundp 'sense_glove_test::glove)
  (if (not (find-package "SENSE_GLOVE_TEST"))
    (make-package "SENSE_GLOVE_TEST"))
  (shadow 'glove (find-package "SENSE_GLOVE_TEST")))
(unless (find-package "SENSE_GLOVE_TEST::GLOVE")
  (make-package "SENSE_GLOVE_TEST::GLOVE"))

(in-package "ROS")
;;//! \htmlinclude glove.msg.html


(defclass sense_glove_test::glove
  :super ros::object
  :slots (_finger _vibration_amplitude _vibration_duration ))

(defmethod sense_glove_test::glove
  (:init
   (&key
    ((:finger __finger) 0)
    ((:vibration_amplitude __vibration_amplitude) 0)
    ((:vibration_duration __vibration_duration) 0)
    )
   (send-super :init)
   (setq _finger (round __finger))
   (setq _vibration_amplitude (round __vibration_amplitude))
   (setq _vibration_duration (round __vibration_duration))
   self)
  (:finger
   (&optional __finger)
   (if __finger (setq _finger __finger)) _finger)
  (:vibration_amplitude
   (&optional __vibration_amplitude)
   (if __vibration_amplitude (setq _vibration_amplitude __vibration_amplitude)) _vibration_amplitude)
  (:vibration_duration
   (&optional __vibration_duration)
   (if __vibration_duration (setq _vibration_duration __vibration_duration)) _vibration_duration)
  (:serialization-length
   ()
   (+
    ;; uint8 _finger
    1
    ;; uint8 _vibration_amplitude
    1
    ;; uint8 _vibration_duration
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _finger
       (write-byte _finger s)
     ;; uint8 _vibration_amplitude
       (write-byte _vibration_amplitude s)
     ;; uint8 _vibration_duration
       (write-byte _vibration_duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _finger
     (setq _finger (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vibration_amplitude
     (setq _vibration_amplitude (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vibration_duration
     (setq _vibration_duration (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sense_glove_test::glove :md5sum-) "ebb0cd83724acf33d4c91ba9d3bf438e")
(setf (get sense_glove_test::glove :datatype-) "sense_glove_test/glove")
(setf (get sense_glove_test::glove :definition-)
      "uint8 finger
uint8 vibration_amplitude
uint8 vibration_duration

")



(provide :sense_glove_test/glove "ebb0cd83724acf33d4c91ba9d3bf438e")


