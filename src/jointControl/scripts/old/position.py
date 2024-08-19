#!/usr/bin/env python3

import cv2
import apriltag
import numpy as np

# AprilTag dedektörü oluştur
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

# Kamera kalibrasyon parametreleri (kalibrasyon kodundan elde edilen değerler)
camera_matrix = np.array(
    [[698.85, 0, 628.36], [0, 698.85, 362.004], [0, 0, 1]]
)  # Örnek değerler
dist_coeffs = np.array(
    [-0.1734, 0.0274, -0.0007, 0.0004, 0]
)  # Bozulma katsayıları (varsayılan olarak sıfır)
tag_size = 0.1  # AprilTag'in gerçek boyutu (metre cinsinden)

cap = cv2.VideoCapture("dev/video2")  # USB/Web kamerayı aç


def get_tag_pose(tag, camera_matrix, dist_coeffs, tag_size):
    # Tag'in köşelerini al
    object_points = np.array(
        [
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0],
        ]
    )

    # Köşeleri OpenCV formatına dönüştür
    image_points = np.array(tag.corners, dtype=np.float32)

    # Poz tahminini hesapla
    _, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )

    # Rotasyon matrisini hesapla
    rmat, _ = cv2.Rodrigues(rvec)

    # Tag'in kameraya göre 3D pozisyonu
    pose = np.hstack((rmat, tvec))

    return pose, rvec, tvec


try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # AprilTag'leri algıla
        tags = detector.detect(gray)

        for tag in tags:
            print(f"Tag ID: {tag.tag_id}, Center: {tag.center}")

            # Tag'in pozunu hesapla
            pose, rvec, tvec = get_tag_pose(tag, camera_matrix, dist_coeffs, tag_size)
            print(f"Tag ID: {tag.tag_id}, Pose:\n{pose}")

            # Tag'i işaretle
            for corner in tag.corners:
                cv2.circle(frame, tuple(int(x) for x in corner), 5, (0, 255, 0), -1)
            cv2.putText(
                frame,
                f"ID: {tag.tag_id}",
                (int(tag.center[0]), int(tag.center[1])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # Görüntüyü göster
        cv2.imshow("Image", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
