import cv2
import numpy as np
from imutils.perspective import four_point_transform

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

count = 0
scale = 0.5

font = cv2.FONT_HERSHEY_SIMPLEX

WIDTH, HEIGHT = 720, 600

lower_blue = np.array([93, 50, 16])
upper_blue = np.array([179, 113, 85])

isWarped = False


def scan_detection(image):
    global document_contour

    document_contour = np.array([[0, 0], [WIDTH, 0], [WIDTH, HEIGHT], [0, HEIGHT]])

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    threshold = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
    )

    contours, _ = cv2.findContours(
        threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.015 * peri, True)
            if area > max_area and len(approx) == 4:
                document_contour = approx
                max_area = area

    cv2.drawContours(frame, [document_contour], -1, (0, 255, 0), 3)
    cv2.imshow("threshold", threshold)


def matrix(Loop_x, Loop_y, frame):
    x, y = 0, 0
    for i in range(Loop_x):
        for j in range(Loop_y):
            # cv2.rectangle(frame,(x + (i * int(WIDTH / 5)),y + (j * int(HEIGHT / 3))),(x + int(WIDTH / 5) + (i * int(WIDTH / 5)),y + int(HEIGHT / 3) + (j * int(HEIGHT / 3))),(255,0,0),2)
            new = frame[
                y + (j * int(HEIGHT / 3)) : y + int(HEIGHT / 3) + (j * int(HEIGHT / 3)),
                x + (i * int(WIDTH / 5)) : x + int(WIDTH / 5) + (i * int(WIDTH / 5)),
            ]
            if j == 0:
                gray = cv2.cvtColor(new, cv2.COLOR_BGR2GRAY)
                _, threshold = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
                # cv2.imshow(f"{i},{j}threshold",threshold)
                contours, _ = cv2.findContours(
                    threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                )
                for contour in contours:
                    area = cv2.contourArea(contour)
                    approx = cv2.approxPolyDP(
                        contour, 0.012 * cv2.arcLength(contour, True), True
                    )
                    bx, by, bw, bh = cv2.boundingRect(contour)
                    if area > 800:
                        if bx < new.shape[1] / 3.25:
                            print(f"1,0 {bx} {i} {new.shape[1]/ 3.25}")
                        elif bx < 2 * new.shape[1] / 3.25:
                            print(f"1, 1 {bx} {i} {2 * new.shape[1]/ 3.25}")
                        elif bx < new.shape[1]:
                            print(f"1, 2 {bx} {i} {new.shape[1]}")
                        else:
                            print(0)
                cv2.imshow(f"{i},{j}threshold", new)

            elif j == 1:
                # cv2.imshow(f"{i},{j}new",new)
                mask = cv2.inRange(new, lower_blue, upper_blue)
                contours, _ = cv2.findContours(
                    mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
                )

                for contour in contours:
                    area = cv2.contourArea(contour)
                    approx = cv2.approxPolyDP(
                        contour, 0.015 * cv2.arcLength(contour, True), True
                    )
                    if area >= 120:
                        # cv2.drawContours(frame, [approx], 0, (0, 0, 0), 2)
                        xx = approx.ravel()[0]
                        yy = approx.ravel()[1] - 5
                        if len(approx) == 3:
                            print(f"{i}Triangle")
                            # cv2.putText(frame, "Triangle", (xx, yy), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                        elif len(approx) == 4:
                            print(f"{i}Square")
                            # cv2.putText(frame, "Square", (xx, yy), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                        elif len(approx) >= 5 and len(approx) <= 7:
                            print(f"{i}Pentagon")
                            # cv2.putText(frame, "Pentagon", (xx, yy), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                        elif len(approx) > 7 and len(approx) < 9:
                            print(f"{i}Cross")
                            # cv2.putText(frame, "Cross", (xx, yy), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                        else:
                            print(f"{i}Circle")
                            # cv2.putText(frame, "Circle", (xx, yy), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

                # cv2.imshow(f"{i},{j}new2", mask)


while True:
    _, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.flip(frame, 2)
    frame = cv2.flip(frame, 1)
    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    frame_copy = frame.copy()

    scan_detection(frame_copy)

    cv2.imshow("input", cv2.resize(frame, (int(scale * WIDTH), int(scale * HEIGHT))))

    if not isWarped:
        warped = four_point_transform(frame_copy, document_contour.reshape(4, 2))
        warped = cv2.resize(warped, (WIDTH, HEIGHT))
        isWarped = True

    if isWarped:
        matrix(5, 3, warped)
        cv2.imshow(
            "Warped",
            cv2.resize(
                warped, (int(scale * warped.shape[1]), int(scale * warped.shape[0]))
            ),
        )

    pressed_key = cv2.waitKey(1) & 0xFF
    if pressed_key == 27:
        break

cv2.destroyAllWindows()
