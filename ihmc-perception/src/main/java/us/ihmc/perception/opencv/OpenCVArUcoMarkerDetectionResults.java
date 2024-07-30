package us.ihmc.perception.opencv;

import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TIntIntHashMap;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_objdetect;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

/**
 * This class provides support for copying the full output from
 * {@link OpenCVArUcoMarkerDetector} so that subsequent operations can
 * happen on different threads.
 */
public class OpenCVArUcoMarkerDetectionResults
{
   private final BytedecoImage inputImage;
   private final MatVector corners;
   private final Mat ids;
   private final MatVector rejectedImagePoints;
   private final Mat cameraMatrix;
   private final Mat distortionCoefficients;
   private final TIntArrayList detectedIDs = new TIntArrayList();
   private final TIntIntHashMap markerIDToCornersIndexMap = new TIntIntHashMap();
   private double timeTakenToDetect = 0.0;

   private transient final Mat objectPoints;
   private transient final BytePointer objectPointsDataPointer;
   private transient final Mat rotationVector;
   private transient final Mat translationVector;
   private transient final Scalar defaultBorderColor;

   private transient final Pose3D markerPose = new Pose3D();

   // This is a temp variable, and is not used to store state
   private transient final FramePose3D tempMarkerPose = new FramePose3D();

   // The weird conversion of axes are because sensor frame is assumed x forward z up, but image frame is z forward, x right. This means z in image is
   // x in sensor, and x in image is -y in sensor.
   private static final RigidBodyTransform sensorToImageTransform = new RigidBodyTransform();
   static
   {
      sensorToImageTransform.appendRollRotation(-Math.PI / 2.0);
      sensorToImageTransform.appendPitchRotation(Math.PI / 2.0);
   }

   public OpenCVArUcoMarkerDetectionResults()
   {
      inputImage = new BytedecoImage(100, 100, opencv_core.CV_8UC3);
      cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      distortionCoefficients = new Mat(1, 4, opencv_core.CV_32FC1);
      distortionCoefficients.ptr(0, 0).putFloat(0.0f);
      distortionCoefficients.ptr(0, 1).putFloat(0.0f);
      distortionCoefficients.ptr(0, 2).putFloat(0.0f);
      distortionCoefficients.ptr(0, 3).putFloat(0.0f);
      distortionCoefficients.ptr(0, 4).putFloat(0.0f);
      corners = new MatVector();
      ids = new Mat();
      rejectedImagePoints = new MatVector();
      objectPoints = new Mat(4, 1, opencv_core.CV_32FC3);
      objectPointsDataPointer = objectPoints.ptr();
      rotationVector = new Mat(3, 1, opencv_core.CV_64FC1);
      translationVector = new Mat(3, 1, opencv_core.CV_64FC1);
      defaultBorderColor = new Scalar(0, 0, 255, 0);
   }

   /**
    * Gets a thread safe copy of all information necessary to make use of the
    * detection results in another thread.
    */
   public void copyOutputData(OpenCVArUcoMarkerDetector detector)
   {
      inputImage.ensureDimensionsMatch(detector.getRGB8ImageForDetection());
      detector.getRGB8ImageForDetection().getBytedecoOpenCVMat().copyTo(inputImage.getBytedecoOpenCVMat());

      corners.clear();
      corners.put(detector.getCorners());

      detector.getIDs().copyTo(ids);

      rejectedImagePoints.clear();
      rejectedImagePoints.put(detector.getRejectedImagePoints());

      detector.getCameraMatrix().copyTo(cameraMatrix);
      detector.getDistortionCoefficients().copyTo(distortionCoefficients);

      detectedIDs.clear();
      markerIDToCornersIndexMap.clear();
      for (int i = 0; i < ids.rows(); i++)
      {
         int markerID = ids.ptr(i, 0).getInt();
         detectedIDs.add(markerID);
         markerIDToCornersIndexMap.put(markerID, i);
      }

      timeTakenToDetect = detector.getDetectionDuration();
   }

   public boolean isDetected(int markerID)
   {
      return markerIDToCornersIndexMap.containsKey(markerID);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Returns whether the pose was successfully found.
    */
   public boolean getPose(int markerID,
                          double markerSize,
                          ReferenceFrame sensorFrame,
                          ReferenceFrame desiredFrame,
                          RigidBodyTransformBasics transformToDesiredFrameToPack)
   {
      return getPose(markerID, markerSize, sensorFrame, desiredFrame, transformToDesiredFrameToPack.getTranslation(), transformToDesiredFrameToPack.getRotation());
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Returns whether the pose was successfully found.
    */
   public boolean getPose(int markerID,
                          double markerSize,
                          ReferenceFrame sensorFrame,
                          ReferenceFrame desiredFrame,
                          Tuple3DBasics translationToPack,
                          Orientation3DBasics orientationToPack)
   {
      boolean success = updateMarkerPose(markerID, markerSize, markerPose);
      tempMarkerPose.setIncludingFrame(sensorFrame, markerPose);
      tempMarkerPose.changeFrame(desiredFrame);
      tempMarkerPose.get(orientationToPack, translationToPack);
      return success;
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Returns whether the pose was successfully found.
    */
   public boolean getPoseInSensorFrame(int markerID, double markerSize, Pose3DBasics poseToPack)
   {
      boolean success = updateMarkerPose(markerID, markerSize, markerPose);
      poseToPack.set(markerPose);
      return success;
   }
   
   /**
    * Estimates the pose of the single marker ID.
    * Multiple markers of the same ID are not supported.
    * Returns whether the pose was successfully found.
    */
   private boolean updateMarkerPose(int markerID, double markerSize, Pose3DBasics poseToPack)
   {
      if (!isDetected(markerID))
      {
         LogTools.debug("Marker ID is invalid.");
         poseToPack.setToNaN();
         return false;
      }

      /**
       * ArUco corner layout:
       * First corner is top left of marker.
       * Secord corner is top right of marker.
       * Third corner is bottom right of marker.
       * Fourth corner is bottom left of marker.
       */
      OpenCVTools.putFloat3(objectPointsDataPointer, 0, 0.0f, (float) -markerSize, (float) markerSize);
      OpenCVTools.putFloat3(objectPointsDataPointer, 1, 0.0f, 0.0f, (float) markerSize);
      OpenCVTools.putFloat3(objectPointsDataPointer, 2, 0.0f, 0.0f, 0.0f);
      OpenCVTools.putFloat3(objectPointsDataPointer, 3, 0.0f, (float) -markerSize, 0.0f);

      int cornersIndex = markerIDToCornersIndexMap.get(markerID);
      long cornersSize = corners.size();
      if (cornersIndex >= cornersSize)
      { // This happens sometimes. There is a bug somewhere, potentially to do with threading, but I can't find it. - @dcalvert
         LogTools.error("Corners index {} is >= the vector size {}. Can't update the pose of this marker this frame.", cornersIndex, cornersSize);
         poseToPack.setToNaN();
         return false;
      }
      else
      {
         Mat markerCorners = corners.get(cornersIndex);
         opencv_calib3d.solvePnP(objectPoints, markerCorners, cameraMatrix, distortionCoefficients, rotationVector, translationVector);

         double rx = rotationVector.ptr(0).getDouble();
         double ry = rotationVector.ptr(0).getDouble(Double.BYTES);
         double rz = rotationVector.ptr(0).getDouble(2 * Double.BYTES);

         double x = translationVector.ptr(0).getDouble();
         double y = translationVector.ptr(0).getDouble(Double.BYTES);
         double z = translationVector.ptr(0).getDouble(2 * Double.BYTES);

         poseToPack.getPosition().set(x, y, z);
         poseToPack.getOrientation().setRotationVector(rx, ry, rz);
         poseToPack.applyTransform(sensorToImageTransform);
         return true;
      }
   }

   public void drawDetectedMarkers(Mat imageForDrawing)
   {
      drawDetectedMarkers(imageForDrawing, defaultBorderColor);
   }

   public void drawDetectedMarkers(Mat imageForDrawing, Scalar borderColor)
   {
      opencv_objdetect.drawDetectedMarkers(imageForDrawing, corners, ids, borderColor);
   }

   public void drawRejectedPoints(Mat imageForDrawing)
   {
      opencv_objdetect.drawDetectedMarkers(imageForDrawing, rejectedImagePoints);
   }

   public BytedecoImage getInputImage()
   {
      return inputImage;
   }

   public Mat getCameraMatrix()
   {
      return cameraMatrix;
   }

   public MatVector getCorners()
   {
      return corners;
   }

   public Mat getIDs()
   {
      return ids;
   }

   public MatVector getRejectedImagePoints()
   {
      return rejectedImagePoints;
   }

   public int getNumberOfRejectedPoints()
   {
      return (int) rejectedImagePoints.size();
   }

   public Mat getObjectPoints()
   {
      return objectPoints;
   }

   public Mat getDistortionCoefficients()
   {
      return distortionCoefficients;
   }

   public TIntArrayList getDetectedIDs()
   {
      return detectedIDs;
   }

   public TIntIntHashMap getMarkerIDToCornersIndexMap()
   {
      return markerIDToCornersIndexMap;
   }

   public double getTimeTakenToDetect()
   {
      return timeTakenToDetect;
   }
}
