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
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;

public class OpenCVArUcoMarkerDetectionOutput
{
   private final TIntArrayList detectedIDs = new TIntArrayList();
   private final MatVector corners;
   private final Mat ids;
   private final MatVector rejectedImagePoints;
   private final Mat cameraMatrix;
   private final Mat distortionCoefficients;
   private final TIntIntHashMap markerIDToCornersIndexMap = new TIntIntHashMap();
   private double timeTakenToDetect = 0.0;

   private transient final Mat objectPoints;
   private transient final BytePointer objectPointsDataPointer;
   private transient final Mat rotationVector;
   private transient final Mat rotationMatrix;
   private transient final Mat translationVector;
   private transient final LinearTransform3D euclidLinearTransform = new LinearTransform3D();
   private transient final Point3D euclidPosition = new Point3D();
   private transient final FramePose3D markerPose = new FramePose3D();
   private transient final Scalar defaultBorderColor;

   public OpenCVArUcoMarkerDetectionOutput()
   {
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
      rotationMatrix = new Mat(3, 3, opencv_core.CV_64FC1);
      translationVector = new Mat(3, 1, opencv_core.CV_64FC1);
      defaultBorderColor = new Scalar(0, 0, 255, 0);
   }

   /**
    * Gets a thread safe copy of all information necessary to make use of the
    * detection results in another thread.
    */
   public void copyOutput(OpenCVArUcoMarkerDetection detection)
   {
      corners.clear();
      corners.put(detection.getCorners());

      ids.copyTo(ids);

      rejectedImagePoints.clear();
      rejectedImagePoints.put(detection.getRejectedImagePoints());

      detection.getCameraMatrix().copyTo(cameraMatrix);
      detection.getDistortionCoefficients().copyTo(distortionCoefficients);

      detectedIDs.clear();
      markerIDToCornersIndexMap.clear();
      for (int i = 0; i < ids.rows(); i++)
      {
         int markerID = ids.ptr(i, 0).getInt();
         detectedIDs.add(markerID);
         markerIDToCornersIndexMap.put(markerID, i);
      }

      timeTakenToDetect = detection.getDetectionDuration();
   }

   public boolean isDetected(int markerID)
   {
      return markerIDToCornersIndexMap.containsKey(markerID);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    */
   public void getPose(int markerID,
                       double markerSize,
                       ReferenceFrame sensorFrame,
                       ReferenceFrame desiredFrame,
                       RigidBodyTransform transformToDesiredFrameToPack)
   {
      updateMarkerPose(markerID, markerSize);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      markerPose.changeFrame(desiredFrame);
      markerPose.get(transformToDesiredFrameToPack);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    */
   public void getPose(int markerID,
                       double markerSize,
                       ReferenceFrame sensorFrame,
                       ReferenceFrame desiredFrame,
                       Point3D translationToPack,
                       Quaternion orientationToPack)
   {
      updateMarkerPose(markerID, markerSize);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      markerPose.changeFrame(desiredFrame);
      markerPose.get(orientationToPack, translationToPack);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    */
   public Pose3DReadOnly getPoseInSensorFrame(int markerID, double markerSize, ReferenceFrame sensorFrame)
   {
      updateMarkerPose(markerID, markerSize);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      return markerPose;
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    */
   public void getPose(int markerID, double markerSize, Pose3DBasics poseToPack)
   {
      updateMarkerPose(markerID, markerSize);
      poseToPack.set(euclidPosition, euclidLinearTransform.getAsQuaternion());
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    */
   public void getPose(int markerID, double markerSize, RigidBodyTransform transformToSensor)
   {
      updateMarkerPose(markerID, markerSize);
      transformToSensor.set(euclidLinearTransform.getAsQuaternion(), euclidPosition);
   }

   /**
    * Estimates the pose of the single marker ID.
    * Multiple markers of the same ID is not supported.
    */
   private void updateMarkerPose(int markerID, double markerSize)
   {
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
         LogTools.error("Corners index {} is >= the vector size {}. Can't update the pose of this marker this frame.",
                        cornersIndex, cornersSize);
      }
      else
      {
         Mat markerCorners = corners.get(cornersIndex);
         opencv_calib3d.solvePnP(objectPoints, markerCorners, cameraMatrix, distortionCoefficients, rotationVector, translationVector);

         // Couldn't figure out why we had to apply these transforms here and below, but it works.
         double rx = rotationVector.ptr(0).getDouble();
         double ry = rotationVector.ptr(0).getDouble(Double.BYTES);
         double rz = rotationVector.ptr(0).getDouble(2 * Double.BYTES);
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
         // These are probably because the coordinate system we define ourselves now for the solvePnP method,
         // probably why they did it,so the way we define it must be different to the way it was internally
         // in estimatePoseSingleMarkers.
         euclidLinearTransform.appendRollRotation(-Math.PI / 2.0);
         euclidLinearTransform.appendPitchRotation(Math.PI / 2.0);

         double x = translationVector.ptr(0).getDouble();
         double y = translationVector.ptr(0).getDouble(Double.BYTES);
         double z = translationVector.ptr(0).getDouble(2 * Double.BYTES);
         euclidPosition.set(z, -x, -y);
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

   public TIntArrayList getDetectedIDs()
   {
      return detectedIDs;
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

   public Mat getObjectPoints()
   {
      return objectPoints;
   }

   public Mat getDistortionCoefficients()
   {
      return distortionCoefficients;
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
