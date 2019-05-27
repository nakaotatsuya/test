#!/usr/bin/env roseus

(ros::load-ros-manifest "std_msgs")

(defun fft-feature-cb (msg)
  (let* ((data (send msg :data)))
    (setq *fft-feature* data)
    )
  )

(defun fft-feature-listen (&optional (iterate 10))
  (let* ((fft-feature-list nil))
    (ros::subscribe "/fft_feature_extraction/output"
		    std_msgs::Float32 #'fft-feature-cb)
    (dotimes (i iterate)
      (setq *fft-feature* nil)
      (ros::spin-once)
      (when *fft-feature*
	(push *fft-feature* fft-feature-list))
      (ros::sleep)
      )
    (sort fft-feature-list #'<)
    (print fft-feature-list)
    (ros::unsubscribe "/fft_feature_extraction/output")
    ;; return the median
    (cond
      ((> (length fft-feature-list) (* 0.5 iterate))
       (setq *fft-feature-fixed* (median fft-feature-list))
       :high)
      ((> (length fft-feature-list) (* 0.1 iterate))
       (setq *fft-feature-fixed* (median fft-feature-list))
       :low)
      (t
       (setq *fft-feature-fixed* nil)
       :no-sound)
      )
    )
  )

(defun median (sequence)
  (let* ((len (length sequence)))
    (if (= (/ len 2) 0)
	(/ (+ (elt sequence (/ len 2))
	      (elt sequence (1+ (/ len 2)))) 2.0)
      (elt sequence (1+ (/ len 2))))))
