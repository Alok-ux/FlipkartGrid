import cv2
import numpy as np
import argparse


def undistort(image, filepath='calibration_data.npz'):
    params = np.load(filepath)
    return cv2.undistort(image, params['intrinsic_matrix'], params['distCoeff'], None)


class CameraCalibration:
    def __init__(self, w: int, h: int, board_dim: int):
        self.board_w = w
        self.board_h = h
        self.image_shape = None

        self.pts_3d = []  # vector for 3d points
        self.pts_2d = []  # vector for 2d points
        self.intrinsic_matrix = np.zeros((3, 3), np.float32)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

        # prepare object points based on the actual dimensions of the calibration board
        # like (0,0,0), (25,0,0), (50,0,0) ....,(200,125,0)
        self.object_pts = np.zeros((h * w, 3), np.float32)
        self.object_pts[:, :2] = np.mgrid[0: (w * board_dim): board_dim, 0: (h * board_dim): board_dim].T.reshape(-1, 2)

    def process_image(self, image):
        cv2.putText(image, 'Process frame: Space \nCalibrate: Enter \nAbort: Esc', (20, image.shape[1]-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 1, cv2.LINE_AA)
        cv2.imshow('image', image)
        key = cv2.waitKey(10) & 0xFF

        if key == ord(chr(27)):      # ESC
            return True

        elif key == ord(chr(13)):    # Enter
            cv2.destroyAllWindows()
            self.calibration(image.shape[:-1])

        elif key == ord(chr(32)):    # Space
            grey_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            found, corners = cv2.findChessboardCorners(grey_image, (self.board_w, self.board_h),
                                                       cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                print("corners found")
                self.get_corners(grey_image, corners)
            else:
                print("false")

    def get_corners(self, grey_image, corners):
        # Add the "true" checkerboard corners
        self.pts_3d.append(self.object_pts)
        self.image_shape = grey_image.shape

        # Improve the accuracy of the checkerboard corners found in the image and save them to the pts_2d variable.
        cv2.cornerSubPix(grey_image, corners, (20, 20), (-1, -1), self.criteria)
        self.pts_2d.append(corners)

        # Draw chessboard corners
        new_img = cv2.cvtColor(grey_image, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(new_img, (self.board_w, self.board_h), corners, True)
        cv2.imshow("corners_detected", new_img)

    def calibration(self, image_shape):
        ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(self.pts_3d, self.pts_2d, image_shape[::-1],
                                                                      None, None)
        print('\nIntrinsic Matrix: \n')
        print(str(matrix))

        print('\nDistortion Coefficients: \n')
        print(str(distortion))

        print('\nSaving data file...')
        np.savez('calibration_data', distCoeff=distortion, intrinsic_matrix=matrix)
        print('Calibration complete')

        # Calculate the total reprojection error.  The closer to zero the better.
        total_error = 0
        for i in range(len(self.pts_3d)):
            img_points_2d, _ = cv2.projectPoints(self.pts_3d[i], r_vecs[i], t_vecs[i], matrix, distortion)
            error = cv2.norm(self.pts_2d[i], img_points_2d, cv2.NORM_L2) / len(img_points_2d)
            total_error += error

        print("total reprojection error: ", total_error / len(self.pts_3d))


def main(args):
    cam_cali = CameraCalibration(args.board_l, args.board_b, args.board_dim)
    source = int(args.source) if args.source.isnumeric() else args.source
    cap = cv2.VideoCapture(source)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            # break
            cap.open(source)
            continue

        if args.undistort:
            cv2.imshow('undistort', undistort(frame, args.filepath))
            if cv2.waitKey(30) & 0xFF == ord(chr(27)):
                break
        else:
            if cam_cali.process_image(frame):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', nargs='?', type=str, default="0")
    parser.add_argument('-b', '--board_b', type=int, default=9, help='number of corners along the row')
    parser.add_argument('-l', '--board_l', type=int, default=6, help='number of corners along the col')
    parser.add_argument('-d', '--board_dim', type=int, default=25, help='board dimensions in cm')
    parser.add_argument('-u', '--undistort', action='store_true', help='undistort function')
    parser.add_argument('-f', '--filepath', type=str, default='calibration_data.npz', help='compressed numpy file')
    main(parser.parse_args())


