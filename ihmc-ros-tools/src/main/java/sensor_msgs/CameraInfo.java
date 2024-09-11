package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface CameraInfo extends Message {
   String _TYPE = "sensor_msgs/CameraInfo";
   String _DEFINITION = "# This message defines meta information for a camera. It should be in a\n# camera namespace on topic \"camera_info\" and accompanied by up to five\n# image topics named:\n#\n#   image_raw - raw data from the camera driver, possibly Bayer encoded\n#   image            - monochrome, distorted\n#   image_color      - color, distorted\n#   image_rect       - monochrome, rectified\n#   image_rect_color - color, rectified\n#\n# The image_pipeline contains packages (image_proc, stereo_image_proc)\n# for producing the four processed image topics from image_raw and\n# camera_info. The meaning of the camera parameters are described in\n# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.\n#\n# The image_geometry package provides a user-friendly interface to\n# common operations using this meta information. If you want to, e.g.,\n# project a 3d point into image coordinates, we strongly recommend\n# using image_geometry.\n#\n# If the camera is uncalibrated, the matrices D, K, R, P should be left\n# zeroed out. In particular, clients may assume that K[0] == 0.0\n# indicates an uncalibrated camera.\n\n#######################################################################\n#                     Image acquisition info                          #\n#######################################################################\n\n# Time of image acquisition, camera coordinate frame ID\nHeader header    # Header timestamp should be acquisition time of image\n                 # Header frame_id should be optical frame of camera\n                 # origin of frame should be optical center of camera\n                 # +x should point to the right in the image\n                 # +y should point down in the image\n                 # +z should point into the plane of the image\n\n\n#######################################################################\n#                      Calibration Parameters                         #\n#######################################################################\n# These are fixed during camera calibration. Their values will be the #\n# same in all messages until the camera is recalibrated. Note that    #\n# self-calibrating systems may \"recalibrate\" frequently.              #\n#                                                                     #\n# The internal parameters can be used to warp a raw (distorted) image #\n# to:                                                                 #\n#   1. An undistorted image (requires D and K)                        #\n#   2. A rectified image (requires D, K, R)                           #\n# The projection matrix P projects 3D points into the rectified image.#\n#######################################################################\n\n# The image dimensions with which the camera was calibrated. Normally\n# this will be the full camera resolution in pixels.\nuint32 height\nuint32 width\n\n# The distortion model used. Supported models are listed in\n# sensor_msgs/distortion_models.h. For most cameras, \"plumb_bob\" - a\n# simple model of radial and tangential distortion - is sufficent.\nstring distortion_model\n\n# The distortion parameters, size depending on the distortion model.\n# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).\nfloat64[] D\n\n# Intrinsic camera matrix for the raw (distorted) images.\n#     [fx  0 cx]\n# K = [ 0 fy cy]\n#     [ 0  0  1]\n# Projects 3D points in the camera coordinate frame to 2D pixel\n# coordinates using the focal lengths (fx, fy) and principal point\n# (cx, cy).\nfloat64[9]  K # 3x3 row-major matrix\n\n# Rectification matrix (stereo cameras only)\n# A rotation matrix aligning the camera coordinate system to the ideal\n# stereo image plane so that epipolar lines in both stereo images are\n# parallel.\nfloat64[9]  R # 3x3 row-major matrix\n\n# Projection/camera matrix\n#     [fx'  0  cx' Tx]\n# P = [ 0  fy' cy' Ty]\n#     [ 0   0   1   0]\n# By convention, this matrix specifies the intrinsic (camera) matrix\n#  of the processed (rectified) image. That is, the left 3x3 portion\n#  is the normal camera intrinsic matrix for the rectified image.\n# It projects 3D points in the camera coordinate frame to 2D pixel\n#  coordinates using the focal lengths (fx', fy') and principal point\n#  (cx', cy') - these may differ from the values in K.\n# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will\n#  also have R = the identity and P[1:3,1:3] = K.\n# For a stereo pair, the fourth column [Tx Ty 0]' is related to the\n#  position of the optical center of the second camera in the first\n#  camera's frame. We assume Tz = 0 so both cameras are in the same\n#  stereo image plane. The first camera always has Tx = Ty = 0. For\n#  the right (second) camera of a horizontal stereo pair, Ty = 0 and\n#  Tx = -fx' * B, where B is the baseline between the cameras.\n# Given a 3D point [X Y Z]', the projection (x, y) of the point onto\n#  the rectified image is given by:\n#  [u v w]' = P * [X Y Z 1]'\n#         x = u / w\n#         y = v / w\n#  This holds for both images of a stereo pair.\nfloat64[12] P # 3x4 row-major matrix\n\n\n#######################################################################\n#                      Operational Parameters                         #\n#######################################################################\n# These define the image region actually captured by the camera       #\n# driver. Although they affect the geometry of the output image, they #\n# may be changed freely without recalibrating the camera.             #\n#######################################################################\n\n# Binning refers here to any camera setting which combines rectangular\n#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the\n#  resolution of the output image to\n#  (width / binning_x) x (height / binning_y).\n# The default values binning_x = binning_y = 0 is considered the same\n#  as binning_x = binning_y = 1 (no subsampling).\nuint32 binning_x\nuint32 binning_y\n\n# Region of interest (subwindow of full camera resolution), given in\n#  full resolution (unbinned) image coordinates. A particular ROI\n#  always denotes the same window of pixels on the camera sensor,\n#  regardless of binning settings.\n# The default setting of roi (all values 0) is considered the same as\n#  full resolution (roi.width = width, roi.height = height).\nRegionOfInterest roi\n";

   Header getHeader();

   void setHeader(Header var1);

   int getHeight();

   void setHeight(int var1);

   int getWidth();

   void setWidth(int var1);

   String getDistortionModel();

   void setDistortionModel(String var1);

   double[] getD();

   void setD(double[] var1);

   double[] getK();

   void setK(double[] var1);

   double[] getR();

   void setR(double[] var1);

   double[] getP();

   void setP(double[] var1);

   int getBinningX();

   void setBinningX(int var1);

   int getBinningY();

   void setBinningY(int var1);

   RegionOfInterest getRoi();

   void setRoi(RegionOfInterest var1);
}