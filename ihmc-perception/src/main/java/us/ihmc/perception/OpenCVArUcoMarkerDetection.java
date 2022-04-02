package us.ihmc.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_aruco.DetectorParameters;
import org.bytedeco.opencv.opencv_aruco.Dictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;

import java.util.ArrayList;

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
   private Mat distanceCoefficient;
   private ArrayList<Integer> idsAsList = new ArrayList<>();

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
      distanceCoefficient = new Mat();

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
                                 distanceCoefficient);
      idsAsList.clear();
      for (int i = 0; i < ids.cols(); i++)
      {
         idsAsList.add(ids.ptr(0, i).getInt());
      }
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
