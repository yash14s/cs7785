#!/usr/bin/env python
import cv2

def main():
    cap = cv2.VideoCapture(0)
    while (True):
        ret, frame = cap.read()
        hsvLower = (90, 100, 50)
        hsvUpper = (150, 255, 255)
        rgb_image = frame
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("hsv_image",hsv_image)
        cv2.waitKey(1)
        binary_image_mask = cv2.inRange(hsv_image, hsvLower, hsvUpper)
        contours, hierarchy = cv2.findContours(binary_image_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if (area > 5000):
                cv2.drawContours(rgb_image, [c], -1, (150, 250, 150), 1)
                # Get center coordinates
                M = cv2.moments(c)
                cx = -1
                cy = -1
                if (M['m00'] != 0):
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print(cx, cy)
                cv2.circle(rgb_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
        cv2.imshow("Output", rgb_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()