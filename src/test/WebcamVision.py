import cv2

camera = cv2.VideoCapture(0)

while True:
    ret, frame = camera.read()
    
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("test", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break