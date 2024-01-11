package us.ihmc.perception.sceneGraph.arUco;

import org.bytedeco.opencv.global.*;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.sceneGraph.SceneObjectDefinitions;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

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
   private final Supplier<ReferenceFrame> blackflyFrameSupplier;

   private final BlackflyLensProperties blackflyLensProperties;
   private GpuMat undistortionMap1;
   private GpuMat undistortionMap2;
   private final Mat cameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);

   private final AtomicBoolean arUcoProcessInitialized = new AtomicBoolean(false);

   public ArUcoDetectionUpdater(ROS2Helper ros2Helper, BlackflyLensProperties blackflyLensProperties, Supplier<ReferenceFrame> blackflyFrameSupplier)
   {
      this.blackflyLensProperties = blackflyLensProperties;
      this.blackflyFrameSupplier = blackflyFrameSupplier;
      this.ros2Helper = ros2Helper;
   }

   public void undistortAndUpdateArUco(RawImage arUcoImage)
   {
      if (arUcoProcessInitialized.get())
      {
         // Convert color from BGR to RGB
         GpuMat imageForUndistortionRGB = new GpuMat(arUcoImage.getImageHeight(), arUcoImage.getImageWidth(), arUcoImage.getOpenCVType());
         opencv_cudaimgproc.cvtColor(arUcoImage.getGpuImageMatrix(), imageForUndistortionRGB, opencv_imgproc.COLOR_BGR2RGB);

         // Undistort image
         GpuMat undistortedImageRGB = new GpuMat(arUcoImage.getImageHeight(), arUcoImage.getImageWidth(), arUcoImage.getOpenCVType());
         opencv_cudawarping.remap(imageForUndistortionRGB, undistortedImageRGB, undistortionMap1, undistortionMap2, opencv_imgproc.INTER_LINEAR);

         // Update the Bytedeco image
         undistortedImageRGB.download(arUcoBytedecoImage.getBytedecoOpenCVMat());

         arUcoMarkerDetector.update();
         arUcoMarkerDetectionResults.copyOutputData(arUcoMarkerDetector);
         arUcoMarkerPublisher.update();

         // Close stuff
         imageForUndistortionRGB.close();
         undistortedImageRGB.close();
         arUcoImage.release();
      }
   }

   public boolean isInitialized()
   {
      return arUcoProcessInitialized.get();
   }

   public void initializeArUcoDetection(int imageWidth, int imageHeight)
   {
      LogTools.info("Initializing ArUco process");
      LogTools.info("Image dimensions: {} x {}", imageWidth, imageHeight);

      initializeImageUndistortion(imageWidth, imageHeight);
      arUcoBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      arUcoMarkerDetector = new OpenCVArUcoMarkerDetector();
      arUcoMarkerDetector.create();
      arUcoMarkerDetector.setSourceImageForDetection(arUcoBytedecoImage);
      cameraMatrixEstimate.copyTo(arUcoMarkerDetector.getCameraMatrix());
      arUcoMarkerDetectionResults = new OpenCVArUcoMarkerDetectionResults();

      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetectionResults,
                                                                ros2Helper,
                                                                SceneObjectDefinitions.ARUCO_MARKER_SIZES,
                                                                blackflyFrameSupplier.get());

      LogTools.info("ArUco process initialized");
      arUcoProcessInitialized.set(true);
   }

   private void initializeImageUndistortion(int imageWidth, int imageHeight)
   {
      Mat cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(blackflyLensProperties.getFocalLengthXForUndistortion());
      cameraMatrix.ptr(1, 1).putDouble(blackflyLensProperties.getFocalLengthYForUndistortion());
      cameraMatrix.ptr(0, 2).putDouble(blackflyLensProperties.getPrincipalPointXForUndistortion());
      cameraMatrix.ptr(1, 2).putDouble(blackflyLensProperties.getPrincipalPointYForUndistortion());
      opencv_core.setIdentity(cameraMatrixEstimate);
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
                                                                       rectificationTransformation,
                                                                       cameraMatrixEstimate,
                                                                       balanceNewFocalLength,
                                                                       undistortedImageSize,
                                                                       fovScaleFocalLengthDivisor);
      // Fisheye undistortion
      // https://docs.opencv.org/4.7.0/db/d58/group__calib3d__fisheye.html#ga167df4b00a6fd55287ba829fbf9913b9
      Mat tempUndistortionMat1 = new Mat();
      Mat tempUndistortionMat2 = new Mat();

      opencv_calib3d.fisheyeInitUndistortRectifyMap(cameraMatrix,
                                                    distortionCoefficients,
                                                    rectificationTransformation,
                                                    cameraMatrixEstimate,
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
