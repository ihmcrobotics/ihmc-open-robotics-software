package us.ihmc.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_aruco.DetectorParameters;
import org.bytedeco.opencv.opencv_aruco.Dictionary;
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
import us.ihmc.tools.thread.ZeroCopySwapReference;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class OpenCVArUcoMarkerDetection
{
   public static final int DEFAULT_DICTIONARY = opencv_aruco.DICT_4X4_100;

   private Dictionary dictionary;
   private BytedecoImage sourceColorImage;
   private boolean alphaRemovalMode;
   private ReferenceFrame sensorFrame;
   private final FramePose3D markerPose = new FramePose3D();
   private final Object inputImageSync = new Object();
   private ZeroCopySwapReference<BytedecoImage> rgb888ColorImage;
   private final Object detectionDataSync = new Object();
   private SwapReference<MatVector> corners;
   private SwapReference<Mat> ids;
   private SwapReference<MatVector> rejectedImagePoints;
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
   private final Point3D euclidPosition = new Point3D();
   private Mat objectPoints;
   private final SwapReference<Stopwatch> stopwatch = new SwapReference<>(() -> new Stopwatch().start());
   private boolean enabled = true;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private int imageWidth;
   private int imageHeight;
   private Scalar defaultBorderColor;

   public void create(BytedecoImage sourceColorImage, CameraPinholeBrown depthCameraIntrinsics, ReferenceFrame sensorFrame)
   {
      this.sourceColorImage = sourceColorImage;

      // ArUco library doesn't support alpha channel being in there
      alphaRemovalMode = sourceColorImage.getBytedecoOpenCVMat().type() == opencv_core.CV_8UC4;
      this.sensorFrame = sensorFrame;
      imageWidth = sourceColorImage.getImageWidth();
      imageHeight = sourceColorImage.getImageHeight();
      rgb888ColorImage = new ZeroCopySwapReference<>(() -> new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3));

      dictionary = opencv_aruco.getPredefinedDictionary(DEFAULT_DICTIONARY);
      corners = new SwapReference<>(MatVector::new);
      ids = new SwapReference<>(Mat::new);
      rejectedImagePoints = new SwapReference<>(MatVector::new);
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
      defaultBorderColor = new Scalar(0, 0, 255, 0);

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
      if (enabled)
      {
         rgb888ColorImage.accessOnHighPriorityThread(rgb888ColorImage ->
         {
            if (alphaRemovalMode)
            {
               opencv_imgproc.cvtColor(sourceColorImage.getBytedecoOpenCVMat(),
                                       rgb888ColorImage.getBytedecoOpenCVMat(),
                                       opencv_imgproc.COLOR_RGBA2RGB);
            }
            else
            {
               sourceColorImage.getBytedecoOpenCVMat().copyTo(rgb888ColorImage.getBytedecoOpenCVMat());
            }
         });

         executorService.clearQueueAndExecute(() ->
         {
            synchronized (inputImageSync)
            {
               stopwatch.getForThreadOne().lap();
               rgb888ColorImage.accessOnLowPriorityThread(rgb888ColorImage ->
               {
                  opencv_aruco.detectMarkers(rgb888ColorImage.getBytedecoOpenCVMat(),
                                             dictionary,
                                             corners.getForThreadOne(),
                                             ids.getForThreadOne(),
                                             detectorParameters,
                                             rejectedImagePoints.getForThreadOne(),
                                             cameraMatrix,
                                             distortionCoefficients);
               });
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

   public long getNumberOfRejectedPoints()
   {
      return rejectedImagePoints.getForThreadTwo().size();
   }

   public void getImageOfDetection(Mat imageToPack)
   {
      rgb888ColorImage.accessOnHighPriorityThread(rgb888ColorImage -> rgb888ColorImage.getBytedecoOpenCVMat().copyTo(imageToPack));
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
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