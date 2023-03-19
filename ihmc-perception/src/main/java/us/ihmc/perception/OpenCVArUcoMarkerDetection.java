package us.ihmc.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TIntIntHashMap;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_objdetect;
import org.bytedeco.opencv.opencv_objdetect.ArucoDetector;
import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import org.bytedeco.opencv.opencv_objdetect.Dictionary;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_objdetect.RefineParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.function.Consumer;

public class OpenCVArUcoMarkerDetection
{
   public static final int DEFAULT_DICTIONARY = opencv_objdetect.DICT_4X4_100;

   private ArucoDetector arucoDetector;
   private Dictionary dictionary;
   private ReferenceFrame sensorFrame;
   private final FramePose3D markerPose = new FramePose3D();
   private BytedecoImage rgb8ImageForDetection;
   private MatVector corners;
   private Mat ids;
   private MatVector rejectedImagePoints;
   private DetectorParameters detectorParameters;
   private Mat cameraMatrix;
   private Mat distortionCoefficients;
   private final TIntArrayList detectedIDs = new TIntArrayList();
   private final TIntIntHashMap markerIDToCornersIndexMap = new TIntIntHashMap();
   private Mat rotationVector;
   private Mat rotationMatrix;
   private final LinearTransform3D euclidLinearTransform = new LinearTransform3D();
   private Mat translationVector;
   private final Point3D euclidPosition = new Point3D();
   private Mat objectPoints;
   private final Stopwatch stopwatch = new Stopwatch().start();
   private boolean enabled = true;
   private Scalar defaultBorderColor;
   private BytedecoImage optionalSourceColorImage;
   private BytePointer objectPointsDataPointer;

   public void create(ReferenceFrame sensorFrame)
   {
      this.sensorFrame = sensorFrame;

      dictionary = opencv_objdetect.getPredefinedDictionary(DEFAULT_DICTIONARY);
      corners = new MatVector(new Mat(1, 1, opencv_core.CV_32FC2));
      ids = new Mat();
      rejectedImagePoints = new MatVector(new Mat(1, 1, opencv_core.CV_32FC2));
      detectorParameters = new DetectorParameters();
      // We don't need refine parameters
      arucoDetector = new ArucoDetector(dictionary, detectorParameters, (RefineParameters) null);
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
         if (rgb8ImageForDetection == null)
         {
            rgb8ImageForDetection = new BytedecoImage(sourceColorImage.getImageWidth(), sourceColorImage.getImageHeight(), opencv_core.CV_8UC3);
         }

         rgb8ImageForDetection.ensureDimensionsMatch(sourceColorImage);

         if (sourceColorImage.getBytedecoOpenCVMat().type() == opencv_core.CV_8UC4)
         {
            // ArUco library doesn't support alpha channel being in there
            opencv_imgproc.cvtColor(sourceColorImage.getBytedecoOpenCVMat(), rgb8ImageForDetection.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGBA2RGB);
         }
         else
         {
            sourceColorImage.getBytedecoOpenCVMat().copyTo(rgb8ImageForDetection.getBytedecoOpenCVMat());
         }

         stopwatch.lap();
         arucoDetector.setDetectorParameters(detectorParameters);
         arucoDetector.detectMarkers(rgb8ImageForDetection.getBytedecoOpenCVMat(),
                                     corners,
                                     ids,
                                     rejectedImagePoints);
         stopwatch.suspend();

         detectedIDs.clear();
         markerIDToCornersIndexMap.clear();
         for (int i = 0; i < ids.rows(); i++)
         {
            int markerID = ids.ptr(i, 0).getInt();
            detectedIDs.add(markerID);
            markerIDToCornersIndexMap.put(markerID, i);
         }
      }
   }

   public void getCopyOfSourceRGBImage(BytedecoImage imageToPack)
   {
      imageToPack.ensureDimensionsMatch(rgb8ImageForDetection);
      rgb8ImageForDetection.getBytedecoOpenCVMat().copyTo(imageToPack.getBytedecoOpenCVMat());
   }

   public boolean isDetected(OpenCVArUcoMarker marker)
   {
      return markerIDToCornersIndexMap.containsKey(marker.getId());
   }

   public FramePose3DBasics getPose(OpenCVArUcoMarker marker)
   {
      updateMarkerPose(marker);
      markerPose.setIncludingFrame(sensorFrame, euclidPosition, euclidLinearTransform.getAsQuaternion());
      return markerPose;
   }

   public void getPose(OpenCVArUcoMarker marker, Pose3DBasics poseToPack)
   {
      updateMarkerPose(marker);
      poseToPack.set(euclidPosition, euclidLinearTransform.getAsQuaternion());
   }

   public void getPose(OpenCVArUcoMarker marker, RigidBodyTransform transformToSensor)
   {
      updateMarkerPose(marker);
      transformToSensor.set(euclidLinearTransform.getAsQuaternion(), euclidPosition);
   }

   /**
    * Estimates the pose of the single marker ID.
    * Multiple markers of the same ID is not supported.
    */
   private void updateMarkerPose(OpenCVArUcoMarker marker)
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
         BytedecoOpenCVTools.putFloat3(objectPointsDataPointer, 0, 0.0f, (float) -marker.getSideLength(), (float) marker.getSideLength());
         BytedecoOpenCVTools.putFloat3(objectPointsDataPointer, 1, 0.0f, 0.0f, (float) marker.getSideLength());
         BytedecoOpenCVTools.putFloat3(objectPointsDataPointer, 2, 0.0f, 0.0f, 0.0f);
         BytedecoOpenCVTools.putFloat3(objectPointsDataPointer, 3, 0.0f, (float) -marker.getSideLength(), 0.0f);

         Mat markerCorners = corners.get(markerIDToCornersIndexMap.get(marker.getId()));
         opencv_calib3d.solvePnP(objectPoints, markerCorners, cameraMatrix, distortionCoefficients, rotationVector, translationVector);

         // ???
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
         // ???
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

   public void forEachDetectedID(Consumer<Integer> idConsumer)
   {
      for (int i = 0; i < detectedIDs.size(); i++)
      {
         idConsumer.accept(detectedIDs.get(i));
      }
   }

   public Mat getCameraMatrix()
   {
      return cameraMatrix;
   }

   public long getNumberOfRejectedPoints()
   {
      return rejectedImagePoints.size();
   }

   public DetectorParameters getDetectorParameters()
   {
      return detectorParameters;
   }

   public double getTimeTakenToDetect()
   {
      return stopwatch.lapElapsed();
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public Mat getIds()
   {
      return ids;
   }
}