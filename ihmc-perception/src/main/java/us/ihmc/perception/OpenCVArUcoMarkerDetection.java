package us.ihmc.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_aruco.DetectorParameters;
import org.bytedeco.opencv.opencv_aruco.Dictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.LinearTransform3D;

import java.util.ArrayList;
import java.util.HashMap;

public class OpenCVArUcoMarkerDetection
{
   public static final int DEFAULT_DICTIONARY = opencv_aruco.DICT_4X4_100;

   private Dictionary dictionary;
   private BytedecoImage rgb888ColorImage;
   private MatVector corners;
   private Mat ids;
   private MatVector rejectedImagePoints;
   private DetectorParameters detectorParameters;
   private Mat cameraMatrix;
   private Mat distortionCoefficients;
   private final ArrayList<Integer> idsAsList = new ArrayList<>();
   private final HashMap<Integer, Mat> idToCornersMap = new HashMap<>();
   private MatVector cornerForPose;
   private Mat rotationVectors;
   private Mat rotationVector;
   private Mat rotationMatrix;
   private final LinearTransform3D euclidLinearTransform = new LinearTransform3D();
   private Mat translationVectors;
   private Mat objectPoints;

   public void create(BytedecoImage rgb888ColorImage, CameraPinholeBrown depthCameraIntrinsics)
   {
      this.rgb888ColorImage = rgb888ColorImage;

      dictionary = opencv_aruco.getPredefinedDictionary(DEFAULT_DICTIONARY);
      corners = new MatVector();
      ids = new Mat();
      rejectedImagePoints = new MatVector();
      detectorParameters = new DetectorParameters();
      detectorParameters.markerBorderBits(2);
      cameraMatrix = new Mat(3, 3, opencv_core.CV_32FC1);
      distortionCoefficients = new Mat(1, 4, opencv_core.CV_32FC1);
      distortionCoefficients.ptr(0, 0).putFloat(0.0f);
      distortionCoefficients.ptr(0, 1).putFloat(0.0f);
      distortionCoefficients.ptr(0, 2).putFloat(0.0f);
      distortionCoefficients.ptr(0, 3).putFloat(0.0f);
      cornerForPose = new MatVector();
      rotationVectors = new Mat();
      rotationVector = new Mat(3, 1, opencv_core.CV_64FC1);
      rotationMatrix = new Mat(3, 3, opencv_core.CV_64FC1);
      translationVectors = new Mat();
      objectPoints = new Mat();

      cameraMatrix.ptr(0, 0).putFloat((float) depthCameraIntrinsics.getFx());
      cameraMatrix.ptr(0, 1).putFloat(0.0f);
      cameraMatrix.ptr(0, 2).putFloat((float) depthCameraIntrinsics.getCx());
      cameraMatrix.ptr(1, 0).putFloat(0.0f);
      cameraMatrix.ptr(1, 1).putFloat((float) depthCameraIntrinsics.getFy());
      cameraMatrix.ptr(1, 2).putFloat((float) depthCameraIntrinsics.getCy());
      cameraMatrix.ptr(2, 0).putFloat(0.0f);
      cameraMatrix.ptr(2, 1).putFloat(0.0f);
      cameraMatrix.ptr(2, 2).putFloat(1.0f);
   }
   
   public void update()
   {
      opencv_aruco.detectMarkers(rgb888ColorImage.getBytedecoOpenCVMat(),
                                 dictionary,
                                 corners,
                                 ids,
                                 detectorParameters,
                                 rejectedImagePoints,
                                 cameraMatrix,
                                 distortionCoefficients);
      idsAsList.clear();
      idToCornersMap.clear();
      for (int i = 0; i < ids.rows(); i++)
      {
         int markerID = ids.ptr(i, 0).getInt();
         idsAsList.add(markerID);
         idToCornersMap.put(markerID, corners.get(i));
      }
   }

   public boolean isDetected(int markerID)
   {
      return idToCornersMap.containsKey(markerID);
   }

   /**
    * Estimates the pose of the single marker ID.
    * Multiple markers of the same ID is not supported.
    */
   public void getPose(int markerID, double markerLength, Pose3DBasics poseToPack)
   {
      cornerForPose.clear();
      cornerForPose.put(idToCornersMap.get(markerID));

      opencv_aruco.estimatePoseSingleMarkers(cornerForPose,
                                             (float) markerLength,
                                             cameraMatrix,
                                             distortionCoefficients,
                                             rotationVectors,
                                             translationVectors,
                                             objectPoints);

      double rx = rotationVectors.ptr(0).getDouble();
      double ry = rotationVectors.ptr(0).getDouble(Double.BYTES);
      double rz = rotationVectors.ptr(0).getDouble(2 * Double.BYTES);
      rotationVector.ptr(0).putDouble(rz);
      rotationVector.ptr(0).putDouble(Double.BYTES, -rx);
      rotationVector.ptr(0).putDouble(2 * Double.BYTES, -ry);

      opencv_calib3d.Rodrigues(rotationVector, rotationMatrix);

      BytePointer basePtr = rotationMatrix.ptr(0);
      euclidLinearTransform.set(basePtr.getDouble(),
                                basePtr.getDouble(Double.BYTES),
                                basePtr.getDouble(2 * Double.BYTES),
                                basePtr.getDouble(3 * Double.BYTES),
                                basePtr.getDouble(4 * Double.BYTES),
                                basePtr.getDouble(5 * Double.BYTES),
                                basePtr.getDouble(6 * Double.BYTES),
                                basePtr.getDouble(7 * Double.BYTES),
                                basePtr.getDouble(8 * Double.BYTES));

      poseToPack.getOrientation().set(euclidLinearTransform.getAsQuaternion());

      double x = translationVectors.ptr(0).getDouble();
      double y = translationVectors.ptr(0).getDouble(Double.BYTES);
      double z = translationVectors.ptr(0).getDouble(2 * Double.BYTES);
      poseToPack.getPosition().set(z, -x, -y);
   }

   public MatVector getCorners()
   {
      return corners;
   }

   public MatVector getRejectedImagePoints()
   {
      return rejectedImagePoints;
   }

   public Mat getIdsAsMat()
   {
      return ids;
   }

   public ArrayList<Integer> getIds()
   {
      return idsAsList;
   }

   public BytedecoImage getImageOfDetection()
   {
      return rgb888ColorImage;
   }

   public DetectorParameters getDetectorParameters()
   {
      return detectorParameters;
   }

   /**
    * Save a ArUco marker image of id to file.
    */
   public static void main(String[] args)
   {
      Mat markerToSave = new Mat();
      Dictionary dictionary = opencv_aruco.getPredefinedDictionary(DEFAULT_DICTIONARY);
      int markerID = 0;
      int totalImageSizePixels = 400;
      for (; markerID < 100; markerID++)
      {
         opencv_aruco.drawMarker(dictionary, markerID, totalImageSizePixels, markerToSave, 2);
         opencv_imgcodecs.imwrite("marker" + markerID + ".jpg", markerToSave);
      }
   }
}
