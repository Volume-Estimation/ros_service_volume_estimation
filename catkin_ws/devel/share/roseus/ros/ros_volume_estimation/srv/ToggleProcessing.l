;; Auto-generated. Do not edit!


(when (boundp 'ros_volume_estimation::ToggleProcessing)
  (if (not (find-package "ROS_VOLUME_ESTIMATION"))
    (make-package "ROS_VOLUME_ESTIMATION"))
  (shadow 'ToggleProcessing (find-package "ROS_VOLUME_ESTIMATION")))
(unless (find-package "ROS_VOLUME_ESTIMATION::TOGGLEPROCESSING")
  (make-package "ROS_VOLUME_ESTIMATION::TOGGLEPROCESSING"))
(unless (find-package "ROS_VOLUME_ESTIMATION::TOGGLEPROCESSINGREQUEST")
  (make-package "ROS_VOLUME_ESTIMATION::TOGGLEPROCESSINGREQUEST"))
(unless (find-package "ROS_VOLUME_ESTIMATION::TOGGLEPROCESSINGRESPONSE")
  (make-package "ROS_VOLUME_ESTIMATION::TOGGLEPROCESSINGRESPONSE"))

(in-package "ROS")





(defclass ros_volume_estimation::ToggleProcessingRequest
  :super ros::object
  :slots (_enabled ))

(defmethod ros_volume_estimation::ToggleProcessingRequest
  (:init
   (&key
    ((:enabled __enabled) nil)
    )
   (send-super :init)
   (setq _enabled __enabled)
   self)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:serialization-length
   ()
   (+
    ;; bool _enabled
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ros_volume_estimation::ToggleProcessingResponse
  :super ros::object
  :slots (_success _message ))

(defmethod ros_volume_estimation::ToggleProcessingResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ros_volume_estimation::ToggleProcessing
  :super ros::object
  :slots ())

(setf (get ros_volume_estimation::ToggleProcessing :md5sum-) "91b5d40c59e3059df49ed9dc7354ebfc")
(setf (get ros_volume_estimation::ToggleProcessing :datatype-) "ros_volume_estimation/ToggleProcessing")
(setf (get ros_volume_estimation::ToggleProcessing :request) ros_volume_estimation::ToggleProcessingRequest)
(setf (get ros_volume_estimation::ToggleProcessing :response) ros_volume_estimation::ToggleProcessingResponse)

(defmethod ros_volume_estimation::ToggleProcessingRequest
  (:response () (instance ros_volume_estimation::ToggleProcessingResponse :init)))

(setf (get ros_volume_estimation::ToggleProcessingRequest :md5sum-) "91b5d40c59e3059df49ed9dc7354ebfc")
(setf (get ros_volume_estimation::ToggleProcessingRequest :datatype-) "ros_volume_estimation/ToggleProcessingRequest")
(setf (get ros_volume_estimation::ToggleProcessingRequest :definition-)
      "bool enabled
---
bool success
string message
")

(setf (get ros_volume_estimation::ToggleProcessingResponse :md5sum-) "91b5d40c59e3059df49ed9dc7354ebfc")
(setf (get ros_volume_estimation::ToggleProcessingResponse :datatype-) "ros_volume_estimation/ToggleProcessingResponse")
(setf (get ros_volume_estimation::ToggleProcessingResponse :definition-)
      "bool enabled
---
bool success
string message
")



(provide :ros_volume_estimation/ToggleProcessing "91b5d40c59e3059df49ed9dc7354ebfc")


