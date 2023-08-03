package us.ihmc.perception.opencv;

import boofcv.struct.calib.CameraPinholeBrown;
import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_objdetect;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_objdetect.ArucoDetector;
import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import org.bytedeco.opencv.opencv_objdetect.Dictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_objdetect.RefineParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.tools.thread.Throttler;

public class OpenCVArUcoMarkerDetection
{
   public static final int DEFAULT_DICTIONARY = opencv_objdetect.DICT_4X4_100;

   private ArucoDetector arucoDetector;
   private Dictionary dictionary;
   private ReferenceFrame sensorFrame;
   private final FramePose3D markerPose = new FramePose3D();
   private SwapReference<OpenCVArUcoMakerDetectionSwapData> detectionSwapReference;
   private DetectorParameters detectorParametersForTuners;
   private Mat cameraMatrix;
   private Mat distortionCoefficients;
   private final TIntArrayList detectedIDs = new TIntArrayList();
   private Mat rotationVector;
   private Mat rotationMatrix;
   private final LinearTransform3D euclidLinearTransform = new LinearTransform3D();
   private Mat translationVector;
   private final Point3D euclidPosition = new Point3D();
   private Mat objectPoints;
   private boolean enabled = true;
   private Scalar defaultBorderColor;
   private BytedecoImage optionalSourceColorImage;
   private BytePointer objectPointsDataPointer;
   private volatile boolean running = true;
   private final Timer timer = new Timer();
   private final Throttler throttler = new Throttler().setFrequency(20.0);

   public void create(ReferenceFrame sensorFrame)
   {
      this.sensorFrame = sensorFrame;

      dictionary = opencv_objdetect.getPredefinedDictionary(DEFAULT_DICTIONARY);
      detectionSwapReference = new SwapReference<>(OpenCVArUcoMakerDetectionSwapData::new);
      detectorParametersForTuners = new DetectorParameters();
      // We don't need refine parameters
      arucoDetector = new ArucoDetector(dictionary, detectionSwapReference.getForThreadOne().getDetectorParameters(), (RefineParameters) null);
      cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      distortionCoefficients = new Mat(1, 4, opencv_core.CV_32FC1);
      distortionCoefficients.ptr(0, 0).putFloat(0.0f);
      distortionCoefficients.ptr(0, 1).putFloat(0.0f);
      distortionCoefficients.ptr(0, 2).putFloat(0.0f);
      distortionCoefficients.ptr(0, 3).putFloat(0.0f);
      distortionCoefficients.ptr(0, 4).putFloat(0.0f);
      rotationVector = new Mat(3, 1, opencv_core.CV_64FC1);
      rotationMatrix = new Mat(3, 3, opencv_core.CV_64FC1);
      translationVector = new Mat(3, 1, opencv_core.CV_64FC1);
      objectPoints = new Mat(4, 1, opencv_core.CV_32FC3);
      objectPointsDataPointer = objectPoints.ptr();
      defaultBorderColor = new Scalar(0, 0, 255, 0);

      ThreadTools.startAsDaemon(this::detectMarkersOnAsynchronousThread, "ArUcoMarkerDetection");
   }

   public void setSourceImageForDetection(BytedecoImage optionalSourceColorImage)
   {
      this.optionalSourceColorImage = optionalSourceColorImage;
   }

   public void setCameraInstrinsics(CameraPinholeBrown depthCameraIntrinsics)
   {
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(depthCameraIntrinsics.getFx());
      cameraMatrix.ptr(1, 1).putDouble(depthCameraIntrinsics.getFy());
      cameraMatrix.ptr(0, 2).putDouble(depthCameraIntrinsics.getCx());
      cameraMatrix.ptr(1, 2).putDouble(depthCameraIntrinsics.getCy());
   }

   public void update()
   {
      update(optionalSourceColorImage);
   }

   public void update(BytedecoImage sourceColorImage)
   {
      if (enabled)
      {
         timer.reset();

         synchronized (detectionSwapReference)
         {
            OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadTwo();

            data.ensureImageInitialized(sourceColorImage.getImageWidth(), sourceColorImage.getImageHeight());
            data.getRgb8ImageForDetection().ensureDimensionsMatch(sourceColorImage);

            if (sourceColorImage.getBytedecoOpenCVMat().type() == opencv_core.CV_8UC4)
            {
               // ArUco library doesn't support alpha channel being in there
               opencv_imgproc.cvtColor(sourceColorImage.getBytedecoOpenCVMat(),
                                       data.getRgb8ImageForDetection().getBytedecoOpenCVMat(),
                                       opencv_imgproc.COLOR_RGBA2RGB);
            }
            else
            {
               sourceColorImage.getBytedecoOpenCVMat().copyTo(data.getRgb8ImageForDetection().getBytedecoOpenCVMat());
            }

            OpenCVArUcoMarkerDetectionParametersTools.copy(detectorParametersForTuners, data.getDetectorParameters());

            detectedIDs.clear();
            data.getMarkerIDToCornersIndexMap().clear();
            for (int i = 0; i < data.getIds().rows(); i++)
            {
               int markerID = data.getIds().ptr(i, 0).getInt();
               detectedIDs.add(markerID);
               data.getMarkerIDToCornersIndexMap().put(markerID, i);
            }
         }
      }
   }

   private void detectMarkersOnAsynchronousThread()
   {
      while (running)
      {
         throttler.waitAndRun();
         if (timer.isRunning(0.5))
         {
            OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadOne();

            if (data.getRgb8ImageForDetection() != null)
            {
               data.getStopwatch().lap();
               arucoDetector.setDetectorParameters(data.getDetectorParameters());
               // detectMarkers is the big slow thing, so we put it on an async thread.
               arucoDetector.detectMarkers(data.getRgb8ImageForDetection().getBytedecoOpenCVMat(),
                                           data.getCorners(),
                                           data.getIds(),
                                           data.getRejectedImagePoints());
               data.getStopwatch().suspend();
               data.setHasDetected(true);
            }

            detectionSwapReference.swap();
         }
      }
   }

   public void getCopyOfSourceRGBImage(BytedecoImage imageToPack)
   {
      BytedecoImage rgb8ImageForDetection = detectionSwapReference.getForThreadTwo().getRgb8ImageForDetection();
      imageToPack.ensureDimensionsMatch(rgb8ImageForDetection);
      rgb8ImageForDetection.getBytedecoOpenCVMat().copyTo(imageToPack.getBytedecoOpenCVMat());
   }

   /**
    * Must be synchronized externally over {@link #getSyncObject()}.
    * @return if the marker is currently detected
    */
   public boolean isDetected(int markerID)
   {
      return detectionSwapReference.getForThreadTwo().getMarkerIDToCornersIndexMap().containsKey(markerID);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Must be synchronized externally over {@link #getSyncObject()}.
    */
   public void getPose(int markerID, double markerSize, ReferenceFrame desiredFrame, RigidBodyTransform transformToDesiredFrameToPack)
   {
      updateMarkerPose(markerID, markerSize);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      markerPose.changeFrame(desiredFrame);
      markerPose.get(transformToDesiredFrameToPack);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Must be synchronized externally over {@link #getSyncObject()}.
    */
   public void getPose(int markerID, double markerSize, ReferenceFrame desiredFrame, Point3D translationToPack, Quaternion orientationToPack)
   {
      updateMarkerPose(markerID, markerSize);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      markerPose.changeFrame(desiredFrame);
      markerPose.get(orientationToPack, translationToPack);
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Must be synchronized externally over {@link #getSyncObject()}.
    */
   public Pose3DReadOnly getPoseInSensorFrame(int markerID, double markerSize)
   {
      updateMarkerPose(markerID, markerSize);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      return markerPose;
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Must be synchronized externally over {@link #getSyncObject()}.
    */
   public void getPose(int markerID, double markerSize, Pose3DBasics poseToPack)
   {
      updateMarkerPose(markerID, markerSize);
      poseToPack.set(euclidPosition, euclidLinearTransform.getAsQuaternion());
   }

   /**
    * Get the pose of an ArUco marker. Use with {@link #isDetected} to make sure
    * the ID is currently detected first.
    * Must be synchronized externally over {@link #getSyncObject()}.
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
      if (enabled)
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

         OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadTwo();
         int cornersIndex = data.getMarkerIDToCornersIndexMap().get(markerID);
         long cornersSize = data.getCorners().size();
         if (cornersIndex >= cornersSize)
         { // This happens sometimes. There is a bug somewhere, potentially to do with threading, but I can't find it. - @dcalvert
            LogTools.error("Corners index {} is >= the vector size {}. Can't update the pose of this marker this frame.",
                           cornersIndex, cornersSize);
         }
         else
         {
            Mat markerCorners = data.getCorners().get(cornersIndex);
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
   }

   public void drawDetectedMarkers(Mat imageForDrawing)
   {
      drawDetectedMarkers(imageForDrawing, defaultBorderColor);
   }

   public void drawDetectedMarkers(Mat imageForDrawing, Scalar borderColor)
   {
      OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadTwo();
      opencv_objdetect.drawDetectedMarkers(imageForDrawing, data.getCorners(), data.getIds(), borderColor);
   }

   public void drawRejectedPoints(Mat imageForDrawing)
   {
      MatVector rejectedImagePoints = detectionSwapReference.getForThreadTwo().getRejectedImagePoints();
      opencv_objdetect.drawDetectedMarkers(imageForDrawing, rejectedImagePoints);
   }

   /**
    * Get the list of detected IDs for this update.
    * Synchronize with {@link #getSyncObject()}.
    */
   public TIntArrayList getDetectedIDs()
   {
      return detectedIDs;
   }

   public void destroy()
   {
      System.out.println("Destroying aruco marker detection");
      running = false;
   }

   public Mat getCameraMatrix()
   {
      return cameraMatrix;
   }

   public long getNumberOfRejectedPoints()
   {
      synchronized (detectionSwapReference)
      {
         MatVector rejectedImagePoints = detectionSwapReference.getForThreadTwo().getRejectedImagePoints();
         return rejectedImagePoints.size();
      }
   }

   public DetectorParameters getDetectorParameters()
   {
      return detectorParametersForTuners;
   }

   public double getTimeTakenToDetect()
   {
      return detectionSwapReference.getForThreadTwo().getStopwatch().lapElapsed();
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   /**
    * Providing this because the classes that use this need different data at different times, which is not thread safe anymore, so you can use this to make sure all the stuff you get from the getters is from the same detection result. To use, do
    * <pre>
    *    synchronized (arUcoMarkerDetection.getSyncObject())
    *    {
    *       arUcoMarkerDetection.getIDsMat()
    *       arUcoMarkerDetection.updateMarkerPose(...)
    *       ...
    *    }
    * </pre>
    */
   public Object getSyncObject()
   {
      return detectionSwapReference;
   }

   /** Detected IDs in raw form. Unless needed, use {@link #getDetectedIDs()} instead. */
   public Mat getIDsMat()
   {
      return detectionSwapReference.getForThreadTwo().getIds();
   }

   public boolean getHasDetected()
   {
      return detectionSwapReference.getForThreadTwo().getHasDetected();
   }
}