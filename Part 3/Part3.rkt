#lang racket
;(planeRANSAC Point_Cloud_1_No_Road_Reduced.xyz 0.1 0.1 1)
;(planeRANSAC Point_Cloud_2_No_Road_Reduced.xyz 0.1 0.1 1)
;(planeRANSAC Point_Cloud_3_No_Road_Reduced.xyz 0.1 0.1 1)

;https://stackoverflow.com/questions/16335454/reading-from-file-using-scheme
(define data "Point_Cloud_1_No_Road_Reduced.xyz")

;Given function to read xyz files. Can't get it to work
(define (readXYZ fileIn)
 (let ((sL (map (lambda s (string-split (car s)))
 (cdr (file->lines fileIn)))))
 (map (lambda (L)
 (map (lambda (s)
 (if (eqv? (string->number s) #f)
 s
(string->number s))) L)) sL)))

;Function to get 3 random points
(define (get3RandPoints Ps)
  (list-ref Ps (random (length Ps))))

;function to create a vector from 3 points
(define (vector P1 P2)
  (list
   (- (car P2) (car P1))
   (- (car (cdr P2)) (car (cdr P1)))
   (- (car (cdr (cdr P2))) (car (cdr (cdr P1))))))

;Function to create a plane from 3 points
(define (plane P1 P2 P3)
  (let ((v1 (vector P1 P2)) (v2 (vector P2 P3)))
    (let ((a (- (* (car (cdr v1)) (car (cdr (cdr v2)))) (* (car (cdr v2)) (car (cdr (cdr v1))))))
          (b (- (* (car v2) (car (cdr (cdr v1)))) (* (car v1) (car (cdr (cdr v2))))))
          (c (- (* (car v1) (car (cdr v2))) (* (car (cdr v1)) (car v2)))))
      (list a b c (- 0 (+ (+ (* a (car P1)) (* b (car (cdr P1)))) (* c (car (cdr (cdr P1))))))))))

;Function to check if a point in within a range of eps from Plane
(define (inRange Plane Point eps)
  (let ((a (car Plane)) (b (car (cdr Plane))) (c (car (cdr (cdr Plane)))) (d (car (cdr (cdr (cdr Plane)))))
                        (x (car Point)) (y (car (cdr Point))) (z (car (cdr (cdr Point)))))
    (let ([d (abs (/ (+ (+ (+ (* a x) (* b y)) (* c z)) d) (+ (+ (* a a) (* b b)) (* c c))))])
      (or (< d eps) (= d eps)))))
  
;Function to find the support of a plane
(define (support plane points eps)
  (cond
    ((null? points) (cons '0 plane))
    (else
     (let ([n (car(support plane (cdr points)))])
       (cond
         ((inRange plane (car points) eps) (cons (+ n 1) plane))
         (else (cons n plane)))))))

;Function to find the dominant plane
(define (dominantPlane Ps k eps)
  (let ([pt (get3RandPoints Ps)])
    (let ([plane (plane (car pt) (car (cdr pt)) (car (cdr (cdr pt))))])
      (let ([planeWsupport1 (support plane Ps eps)])
        (cond
          ((= k 0) planeWsupport1)
          (else
           (let ([planeWsupport2 (dominantPlane Ps (- k 1))])
             (cond
               ((< (car planeWsupport1) (car planeWsupport2)) planeWsupport2)
               (else
                (planeWsupport1))))))))))

;Function to calculate the number of iterations necessary for ransac
(define (ransacNumberOfIteration confidence percentage)
  (/ (log (-1 confidence)) (log (- 1 (* (* percentage percentage) percentage)))))

;Function to do ransac and save it's output into a file
(define (planeRANSAC filename confidence percentage eps)
  (let ([points (readXYZ filename)] [k (ransacNumberOfIteration confidence percentage)])
    (dominantPlane points k eps)))
    