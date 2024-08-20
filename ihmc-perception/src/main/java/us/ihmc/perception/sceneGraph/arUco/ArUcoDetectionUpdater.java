package us.ihmc.perception.sceneGraph.arUco;

import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;

/**
 * This class orchestrates:
 * - Undistorting the fisheye input images
 * - Performing ArUco marker detection
 * - Filtering and publishing the results on the ROS 2 visualizer topic
 *
 * This class should not access or have reference to the scene graph.
 */
public class ArUcoDetectionUpdater
{
   private final ROS2Helper ros2Helper;

   private OpenCVArUcoMarkerDetector arUcoMarkerDetector;
   private OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;
   private BytedecoImage arUcoBytedecoImage;
   private final ReferenceFrame sensorFrame;

   private BlackflyLensProperties blackflyLensProperties;
   private GpuMat undistortionMap1;
   private GpuMat undistortionMap2;
   private final Mat undistortedCameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);
   private final TypedNotification<RawImage> inputImage = new TypedNotification<>();

   public ArUcoDetectionUpdater(ROS2Helper ros2Helper, ReferenceFrame sensorFrame)
   {
      this.sensorFrame = sensorFrame;
      this.ros2Helper = ros2Helper;
   }

   public void setUpUndistortion(BlackflyLensProperties blackflyLensProperties)
   {
      this.blackflyLensProperties = blackflyLensProperties;
   }

   public void setNextImage(RawImage arUcoImage)
   {
      arUcoImage.get();

      if (inputImage.poll())
         inputImage.read().release();

      inputImage.set(arUcoImage);
   }

   public boolean updateArUco()
   {
      boolean newImageAvailable = inputImage.poll();
      if (newImageAvailable)
      {
         RawImage arUcoImage = inputImage.read();

         if (arUcoBytedecoImage == null)
            initialize(arUcoImage);

         // Convert color from BGR to RGB
         GpuMat imageForDetection = new GpuMat(arUcoImage.getImageSize(), arUcoImage.getOpenCVType());
         opencv_cudaimgproc.cvtColor(arUcoImage.getGpuImageMat(), imageForDetection, opencv_imgproc.COLOR_BGR2RGB);

         if (blackflyLensProperties != null)
         {
            // Undistort image
            opencv_cudawarping.remap(imageForDetection, imageForDetection, undistortionMap1, undistortionMap2, opencv_imgproc.INTER_LINEAR);
         }

         // Update the Bytedeco image
         imageForDetection.download(arUcoBytedecoImage.getBytedecoOpenCVMat());

         arUcoMarkerDetector.update();
         arUcoMarkerDetectionResults.copyOutputData(arUcoMarkerDetector);
         arUcoMarkerPublisher.update();

         // Close stuff
         imageForDetection.close();
         arUcoImage.release();
      }
      return newImageAvailable;
   }

   private void initialize(RawImage detectionImage)
   {
      int imageWidth = detectionImage.getImageWidth();
      int imageHeight = detectionImage.getImageHeight();

      LogTools.info("Initializing ArUco process");
      LogTools.info("Image dimensions: {} x {}", imageWidth, imageHeight);

      arUcoBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      arUcoMarkerDetector = new OpenCVArUcoMarkerDetector();
      arUcoMarkerDetector.setSourceImageForDetection(arUcoBytedecoImage);
      arUcoMarkerDetectionResults = new OpenCVArUcoMarkerDetectionResults();

      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetectionResults,
                                                                ros2Helper,
                                                                SceneObjectDefinitions.ARUCO_MARKER_SIZES,
                                                                sensorFrame);

      if (blackflyLensProperties != null)
      {
         initializeImageUndistortion(imageWidth, imageHeight);
         undistortedCameraMatrixEstimate.copyTo(arUcoMarkerDetector.getCameraMatrix());
      }
      else
      {
         arUcoMarkerDetector.setCameraIntrinsics(detectionImage.getIntrinsicsCopy());
      }

      LogTools.info("ArUco process initialized");
   }

   private void initializeImageUndistortion(int imageWidth, int imageHeight)
   {
      Mat cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(blackflyLensProperties.getFocalLengthXForUndistortion());
      cameraMatrix.ptr(1, 1).putDouble(blackflyLensProperties.getFocalLengthYForUndistortion());
      cameraMatrix.ptr(0, 2).putDouble(blackflyLensProperties.getPrincipalPointXForUndistortion());
      cameraMatrix.ptr(1, 2).putDouble(blackflyLensProperties.getPrincipalPointYForUndistortion());
      opencv_core.setIdentity(undistortedCameraMatrixEstimate);
      Mat distortionCoefficients = new Mat(blackflyLensProperties.getK1ForUndistortion(),
                                           blackflyLensProperties.getK2ForUndistortion(),
                                           blackflyLensProperties.getK3ForUndistortion(),
                                           blackflyLensProperties.getK4ForUndistortion());
      Size sourceImageSize = new Size(imageWidth, imageHeight);
      Size undistortedImageSize = new Size((int) (SensorHeadParameters.UNDISTORTED_IMAGE_SCALE * imageWidth),
                                           (int) (SensorHeadParameters.UNDISTORTED_IMAGE_SCALE * imageHeight));
      Mat rectificationTransformation = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(rectificationTransformation);

      double balanceNewFocalLength = 0.0;
      double fovScaleFocalLengthDivisor = 1.0;

      opencv_calib3d.fisheyeEstimateNewCameraMatrixForUndistortRectify(cameraMatrix,
                                                                       distortionCoefficients,
                                                                       sourceImageSize,
                                                                       rectificationTransformation, undistortedCameraMatrixEstimate,
                                                                       balanceNewFocalLength,
                                                                       undistortedImageSize,
                                                                       fovScaleFocalLengthDivisor);
      // Fisheye undistortion
      // https://docs.opencv.org/4.7.0/db/d58/group__calib3d__fisheye.html#ga167df4b00a6fd55287ba829fbf9913b9
      Mat tempUndistortionMat1 = new Mat();
      Mat tempUndistortionMat2 = new Mat();

      opencv_calib3d.fisheyeInitUndistortRectifyMap(cameraMatrix,
                                                    distortionCoefficients,
                                                    rectificationTransformation, undistortedCameraMatrixEstimate,
                                                    undistortedImageSize,
                                                    opencv_core.CV_32F,
                                                    tempUndistortionMat1,
                                                    tempUndistortionMat2);

      undistortionMap1 = new GpuMat();
      undistortionMap1.upload(tempUndistortionMat1);
      undistortionMap2 = new GpuMat();
      undistortionMap2.upload(tempUndistortionMat2);

      // Close pointers
      tempUndistortionMat2.close();
      tempUndistortionMat1.close();
      rectificationTransformation.close();
      undistortedImageSize.close();
      sourceImageSize.close();
      distortionCoefficients.close();
      cameraMatrix.close();
   }

   public OpenCVArUcoMarkerDetector getArUcoMarkerDetector()
   {
      return arUcoMarkerDetector;
   }
}
