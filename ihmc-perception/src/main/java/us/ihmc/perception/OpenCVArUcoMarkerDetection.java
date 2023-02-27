package us.ihmc.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_aruco.DetectorParameters;
import org.bytedeco.opencv.opencv_aruco.Dictionary;
import org.bytedeco.opencv.opencv_aruco.EstimateParameters;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class OpenCVArUcoMarkerDetection
{
   public static final int DEFAULT_DICTIONARY = opencv_aruco.DICT_4X4_100;

   private Dictionary dictionary;
   private ReferenceFrame sensorFrame;
   private final FramePose3D markerPose = new FramePose3D();
   private final Object inputImageSync = new Object();
   private SwapReference<BytedecoImage> rgb8ImageForDetection;
   private final Object detectionDataSync = new Object();
   private SwapReference<MatVector> corners;
   private SwapReference<Mat> ids;
   private SwapReference<MatVector> rejectedImagePoints;
   private DetectorParameters detectorParameters;
   private EstimateParameters estimateParameters;
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
   private final Point3D euclidPosition = new Point3D();
   private Mat objectPoints;
   private final SwapReference<Stopwatch> stopwatch = new SwapReference<>(() -> new Stopwatch().start());
   private boolean enabled = true;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private Scalar defaultBorderColor;
   private BytedecoImage optionalSourceColorImage;

   public void create(ReferenceFrame sensorFrame)
   {
      this.sensorFrame = sensorFrame;

      dictionary = opencv_aruco.getPredefinedDictionary(DEFAULT_DICTIONARY);
      corners = new SwapReference<>(MatVector::new);
      ids = new SwapReference<>(Mat::new);
      rejectedImagePoints = new SwapReference<>(MatVector::new);
      detectorParameters = DetectorParameters.create();
      detectorParameters.markerBorderBits(2);
      estimateParameters = EstimateParameters.create();
      cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
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
            rgb8ImageForDetection = new SwapReference<>(() -> new BytedecoImage(sourceColorImage.getImageWidth(),
                                                                                sourceColorImage.getImageHeight(),
                                                                                opencv_core.CV_8UC3));
         }

         synchronized (rgb8ImageForDetection)
         {
            rgb8ImageForDetection.getForThreadOne().ensureDimensionsMatch(sourceColorImage);

            if (sourceColorImage.getBytedecoOpenCVMat().type() == opencv_core.CV_8UC4)
            {
               // ArUco library doesn't support alpha channel being in there
               opencv_imgproc.cvtColor(sourceColorImage.getBytedecoOpenCVMat(),
                                       rgb8ImageForDetection.getForThreadOne().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGBA2RGB);
            }
            else
            {
               sourceColorImage.getBytedecoOpenCVMat().copyTo(rgb8ImageForDetection.getForThreadOne().getBytedecoOpenCVMat());
            }
         }

         // TODO: Throttle this, extract lambda
         executorService.clearQueueAndExecute(() ->
         {
            synchronized (inputImageSync)
            {
               stopwatch.getForThreadOne().lap(); // TODO: Stopwatch swap reference is done incorrectly
               opencv_aruco.detectMarkers(rgb8ImageForDetection.getForThreadTwo().getBytedecoOpenCVMat(),
                                          dictionary,
                                          corners.getForThreadOne(),
                                          ids.getForThreadOne(),
                                          detectorParameters,
                                          rejectedImagePoints.getForThreadOne());
               rgb8ImageForDetection.swap();
               stopwatch.getForThreadOne().suspend();
            }

            synchronized (detectionDataSync)
            {
               corners.swap();
               ids.swap();
               rejectedImagePoints.swap();
               stopwatch.swap();

               idsAsList.clear();
               idToCornersMap.clear();
               for (int i = 0; i < ids.getForThreadTwo().rows(); i++)
               {
                  int markerID = ids.getForThreadTwo().ptr(i, 0).getInt();
                  idsAsList.add(markerID);
                  idToCornersMap.put(markerID, corners.getForThreadTwo().get(i));
               }
            }
         });
      }
   }

   public void getCopyOfSourceRGBImage(BytedecoImage imageToPack)
   {
      synchronized (rgb8ImageForDetection)
      {
         imageToPack.ensureDimensionsMatch(rgb8ImageForDetection.getForThreadOne());
         rgb8ImageForDetection.getForThreadOne().getBytedecoOpenCVMat().copyTo(imageToPack.getBytedecoOpenCVMat());
      }
   }

   public boolean isDetected(OpenCVArUcoMarker marker)
   {
      synchronized (detectionDataSync)
      {
         return idToCornersMap.containsKey(marker.getId());
      }
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
         cornerForPose.clear();

         synchronized (detectionDataSync)
         {
            try
            {
               cornerForPose.put(idToCornersMap.get(marker.getId()));
            }
            catch (NullPointerException nullPointerException)
            {
               LogTools.error(nullPointerException.getMessage());
               return;
            }
         }

         opencv_aruco.estimatePoseSingleMarkers(cornerForPose,
                                                (float) marker.getSideLength(),
                                                cameraMatrix,
                                                distortionCoefficients,
                                                rotationVectors,
                                                translationVectors,
                                                objectPoints,
                                                estimateParameters);

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

         double x = translationVectors.ptr(0).getDouble();
         double y = translationVectors.ptr(0).getDouble(Double.BYTES);
         double z = translationVectors.ptr(0).getDouble(2 * Double.BYTES);
         euclidPosition.set(z, -x, -y);
      }
   }

   public void drawDetectedMarkers(Mat imageForDrawing)
   {
      drawDetectedMarkers(imageForDrawing, defaultBorderColor);
   }

   public void drawDetectedMarkers(Mat imageForDrawing, Scalar borderColor)
   {
      synchronized (detectionDataSync)
      {
         opencv_aruco.drawDetectedMarkers(imageForDrawing, corners.getForThreadTwo(), ids.getForThreadTwo(), borderColor);
      }
   }

   public void drawRejectedPoints(Mat imageForDrawing)
   {
      synchronized (detectionDataSync)
      {
         opencv_aruco.drawDetectedMarkers(imageForDrawing, rejectedImagePoints.getForThreadTwo());
      }
   }

   public void forEachDetectedID(Consumer<Integer> idConsumer)
   {
      synchronized (detectionDataSync)
      {
         for (Integer id : idsAsList)
         {
            idConsumer.accept(id);
         }
      }
   }

   public Mat getCameraMatrix()
   {
      return cameraMatrix;
   }

   public long getNumberOfRejectedPoints()
   {
      return rejectedImagePoints.getForThreadTwo().size();
   }

   public DetectorParameters getDetectorParameters()
   {
      return detectorParameters;
   }

   public double getTimeTakenToDetect()
   {
      return stopwatch.getForThreadTwo().lapElapsed();
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public SwapReference<Mat> getIds()
   {
      return ids;
   }
}