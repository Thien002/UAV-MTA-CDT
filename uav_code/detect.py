import cv2
from ultralytics import YOLO
import time

model = YOLO("best.pt")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
video_name = time.strftime("videos/%Y_%m_%d_%H_%M.mp4")
out = cv2.VideoWriter(video_name,cv2.VideoWriter_fourcc(*"mp4v"),16.0,(640, 360))

def detect():
    if not hasattr(detect, "frame_id"):
        detect.frame_id = 0
        detect.prev_t = 0.0 
        detect.fps_now = 0
    dxc = dyc = None
    best_conf = None
    obj_cx = obj_cy = None
    bbox = None
    ret, frame_ = cap.read()
    if not ret:
        return 0,0,0
    frame = cv2.resize(frame_, (640, 360), interpolation=cv2.INTER_LINEAR)
    cam_cx, cam_cy = 320.0, 180.0  

    r = model.predict(frame, conf=0.5, imgsz=320, device="cuda", half=True, verbose=False)[0]
    b = r.boxes
    if b is not None and len(b) > 0:
        best_conf = float(b.conf[0].item())
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        bbox = (x1, y1, x2, y2)

        obj_cx = (x1 + x2) / 2.0 #tâm object
        obj_cy = (y1 + y2) / 2.0 #tâm object
        dxc = obj_cx - cam_cx #độ lệch theo phương ngang tọa độ camera
        dyc = cam_cy - obj_cy #độ lệch theo phương dọc tọa độ camera
  
    #Tính fps
    detect.frame_id+=1
    if detect.frame_id == 1:
        detect.prev_t = time.time()
    elif detect.frame_id == 10:
        t_now = time.time()
        detect.fps_now = int(9/(t_now - detect.prev_t))
        detect.frame_id = 0
    
    #Vẽ chú thích lên khung ảnh
    annotated = frame
    cv2.putText(annotated, f"FPS:{detect.fps_now}",(10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255),2) #in fps
    cv2.drawMarker(annotated, (int(cam_cx), int(cam_cy)), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2) # tâm camera (vàng)
    
    if bbox is not None:
        x1, y1, x2, y2 = bbox

        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (200, 0, 0), 2) #bbox
        cv2.circle(annotated, (int(obj_cx), int(obj_cy)), 4, (0, 0, 255), -1) # tâm object (đỏ)
        cv2.line(annotated, (int(cam_cx), int(cam_cy)), (int(obj_cx), int(obj_cy)), (0, 255, 255),2) # line nối 2 tâm (vàng)

        # text dx/dy + conf
        cv2.putText(annotated, f"dx={dyc:.1f}px dy={dxc:.1f}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(annotated, f"conf={best_conf:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    if out is not None:
        out.write(annotated)
    return dyc,dxc,best_conf

def close():
    cap.release()
    out.release()

if __name__ == "__main__":
    try:
        while True:
            dx,dy,best_conf = detect()
    except:
        pass
    finally:
        close()