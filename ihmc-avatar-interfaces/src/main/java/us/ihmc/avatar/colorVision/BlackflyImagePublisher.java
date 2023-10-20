package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.*;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sceneGraph.arUco.ArUcoSceneTools;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class BlackflyImagePublisher
{
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final ROS2SceneGraph sceneGraph;
   private final ROS2StoredPropertySet<IntrinsicCameraMatrixProperties> ousterFisheyeColoringIntrinsicsROS2;
   private final IHMCROS2Publisher<ImageMessage> ros2DistoredImagePublisher;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerPublisher;

   private final BlackflyLensProperties lensProperties;
   private final IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics;

   private final CUDAImageEncoder imageEncoder = new CUDAImageEncoder();
   private GpuMat undistortionMap1;
   private GpuMat undistortionMap2;
   private int imageWidth;
   private int imageHeight;
   private BytedecoImage undistortedBytedecoImage;
   private final Supplier<ReferenceFrame> rightBlackflyCameraFrameSupplier;
   private final MutableReferenceFrame blackflyFrameForSceneNodeUpdate = new MutableReferenceFrame();

   private long lastImageSequenceNumber = -1L;
   private RawImage nextGpuDistortedImage;

   private final RestartableThread publishDistoredColorThread;
   private final RestartableThread undistortAndUpdateArUcoThread;
   private final Lock imagePublishLock = new ReentrantLock();
   private final Lock updateArUcoLock = new ReentrantLock();
   private final Condition newImageAvailable = imagePublishLock.newCondition();
   private final Condition updateArUcoCondition = updateArUcoLock.newCondition();

   public BlackflyImagePublisher(BlackflyLensProperties lensProperties, Supplier<ReferenceFrame> rightBlackflyCameraFrameSupplier, ROS2Topic<ImageMessage> distortedImageTopic)
   {
      this.lensProperties = lensProperties;
      this.ousterFisheyeColoringIntrinsics = SensorHeadParameters.loadOusterFisheyeColoringIntrinsicsOnRobot(lensProperties);
      this.rightBlackflyCameraFrameSupplier = rightBlackflyCameraFrameSupplier;

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_publisher");
      ros2Helper = new ROS2Helper(ros2Node);
      sceneGraph = new ROS2SceneGraph(ros2Helper);
      ros2DistoredImagePublisher = ROS2Tools.createPublisher(ros2Node, distortedImageTopic, ROS2QosProfile.BEST_EFFORT());
      ousterFisheyeColoringIntrinsicsROS2 = new ROS2StoredPropertySet<>(ros2Helper,
                                                                        DualBlackflyComms.OUSTER_FISHEYE_COLORING_INTRINSICS,
                                                                        ousterFisheyeColoringIntrinsics);

      publishDistoredColorThread = new RestartableThread("BlackflyDistortedImagePublisher", () ->
      {
         imagePublishLock.lock();
         try
         {
            while (nextGpuDistortedImage == null || nextGpuDistortedImage.isEmpty() || nextGpuDistortedImage.getSequenceNumber() == lastImageSequenceNumber)
            {
               newImageAvailable.await();
            }

            publishDistortedColor(nextGpuDistortedImage);
         }
         catch (InterruptedException interruptedException)
         {
            interruptedException.printStackTrace();
            imagePublishLock.unlock();
         }
      });

      undistortAndUpdateArUcoThread = new RestartableThread("BlackflyUndistortAndUpdateArUco", () ->
      {
         updateArUcoLock.lock();
         try
         {
            while (nextGpuDistortedImage == null || nextGpuDistortedImage.isEmpty() || nextGpuDistortedImage.getSequenceNumber() == lastImageSequenceNumber)
            {
               updateArUcoCondition.await();
            }

            undistortAndUpdateArUco(nextGpuDistortedImage);
         }
         catch (InterruptedException interruptedException)
         {
            interruptedException.printStackTrace();
            updateArUcoLock.unlock();
         }
      });
   }

   private void publishDistortedColor(RawImage distortedImageToPublish)
   {
      RawImage imageToPublish = null;

      if (distortedImageToPublish != null)
         imageToPublish = new RawImage(distortedImageToPublish);
      imagePublishLock.unlock();

      if (imageToPublish != null && !imageToPublish.isEmpty() && imageToPublish.getSequenceNumber() != lastImageSequenceNumber)
      {
         // Compress image
         BytePointer distortedImageJPEGPointer = new BytePointer((long) imageToPublish.getImageHeight() * imageToPublish.getImageWidth());
         imageEncoder.encodeBGR(imageToPublish.getGpuImageMatrix().data(),
                                distortedImageJPEGPointer,
                                imageToPublish.getImageWidth(),
                                imageToPublish.getImageHeight(),
                                imageToPublish.getGpuImageMatrix().step());
         
         // Publish intrinsics
         ousterFisheyeColoringIntrinsicsROS2.updateAndPublishThrottledStatus();
         
         // Publish compressed image
         ImageMessage distortedImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(distortedImageJPEGPointer.limit());
         imageMessageDataPacker.pack(distortedImageMessage, distortedImageJPEGPointer);
         MessageTools.toMessage(imageToPublish.getAcquisitionTime(), distortedImageMessage.getAcquisitionTime());
         distortedImageMessage.setFocalLengthXPixels(imageToPublish.getFocalLengthX());
         distortedImageMessage.setFocalLengthYPixels(imageToPublish.getFocalLengthY());
         distortedImageMessage.setPrincipalPointXPixels(imageToPublish.getPrincipalPointX());
         distortedImageMessage.setPrincipalPointYPixels(imageToPublish.getPrincipalPointY());
         distortedImageMessage.setImageWidth(imageToPublish.getImageWidth());
         distortedImageMessage.setImageHeight(imageToPublish.getImageHeight());
         distortedImageMessage.getPosition().set(imageToPublish.getPosition());
         distortedImageMessage.getOrientation().set(imageToPublish.getOrientation());
         distortedImageMessage.setSequenceNumber(imageToPublish.getSequenceNumber());
         CameraModel.EQUIDISTANT_FISHEYE.packMessageFormat(distortedImageMessage);
         ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(distortedImageMessage);
         ros2DistoredImagePublisher.publish(distortedImageMessage);

         lastImageSequenceNumber = imageToPublish.getSequenceNumber();

         // Close stuff
         distortedImageJPEGPointer.close();
         imageToPublish.destroy();
      }
   }

   private void undistortAndUpdateArUco(RawImage distortedImage)
   {
      RawImage imageForUndistortion = null;
      if (distortedImage != null)
         imageForUndistortion = new RawImage(distortedImage);
      updateArUcoLock.unlock();

      if (imageForUndistortion != null && !imageForUndistortion.isEmpty() && imageForUndistortion.getSequenceNumber() != lastImageSequenceNumber)
      {
         blackflyFrameForSceneNodeUpdate.getTransformToParent().set(rightBlackflyCameraFrameSupplier.get().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
         blackflyFrameForSceneNodeUpdate.getReferenceFrame().update();

         // Convert color from BGR to RGB
         GpuMat imageForUndistortionRGB = new GpuMat(imageForUndistortion.getImageHeight(),
                                                     imageForUndistortion.getImageWidth(),
                                                     imageForUndistortion.getOpenCVType());
         opencv_cudaimgproc.cvtColor(imageForUndistortion.getGpuImageMatrix(), imageForUndistortionRGB, opencv_imgproc.COLOR_BGR2RGB);
         
         // Undistort image
         GpuMat undistortedImageRGB = new GpuMat(imageForUndistortion.getImageHeight(),
                                                 imageForUndistortion.getImageWidth(),
                                                 imageForUndistortion.getOpenCVType());
         opencv_cudawarping.remap(imageForUndistortionRGB, undistortedImageRGB, undistortionMap1, undistortionMap2, opencv_imgproc.INTER_LINEAR);
         undistortedImageRGB.download(undistortedBytedecoImage.getBytedecoOpenCVMat());

         arUcoMarkerDetection.update();
         arUcoMarkerPublisher.update();

         sceneGraph.updateSubscription();
         ArUcoSceneTools.updateSceneGraph(arUcoMarkerDetection, sceneGraph);
         sceneGraph.updateOnRobotOnly(blackflyFrameForSceneNodeUpdate.getReferenceFrame());
         sceneGraph.updatePublication();
      }
   }

   private void initializeImageUndistortion()
   {
      Mat cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(lensProperties.getFocalLengthXForUndistortion());
      cameraMatrix.ptr(1, 1).putDouble(lensProperties.getFocalLengthYForUndistortion());
      cameraMatrix.ptr(0, 2).putDouble(lensProperties.getPrincipalPointXForUndistortion());
      cameraMatrix.ptr(1, 2).putDouble(lensProperties.getPrincipalPointYForUndistortion());
      Mat newCameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);
      opencv_core.setIdentity(newCameraMatrixEstimate);
      Mat distortionCoefficients = new Mat(lensProperties.getK1ForUndistortion(),
                                           lensProperties.getK2ForUndistortion(),
                                           lensProperties.getK3ForUndistortion(),
                                           lensProperties.getK4ForUndistortion());
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
                                                                       newCameraMatrixEstimate,
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
                                                    newCameraMatrixEstimate,
                                                    undistortedImageSize,
                                                    opencv_core.CV_32F,
                                                    tempUndistortionMat1,
                                                    tempUndistortionMat2);

      undistortionMap1 = new GpuMat();
      undistortionMap1.upload(tempUndistortionMat1);
      undistortionMap2 = new GpuMat();
      undistortionMap2.upload(tempUndistortionMat2);

      // Create arUco marker detection
      undistortedBytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(rightBlackflyCameraFrameSupplier.get());
      arUcoMarkerDetection.setSourceImageForDetection(undistortedBytedecoImage);
      newCameraMatrixEstimate.copyTo(arUcoMarkerDetection.getCameraMatrix());
      arUcoMarkerPublisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection, ros2Helper, sceneGraph.getArUcoMarkerIDToNodeMap());

      // Close pointers
      tempUndistortionMat2.close();
      tempUndistortionMat1.close();
      rectificationTransformation.close();
      undistortedImageSize.close();
      sourceImageSize.close();
      distortionCoefficients.close();
      newCameraMatrixEstimate.close();
      cameraMatrix.close();
   }

   public void startImagePublishing()
   {
      publishDistoredColorThread.start();
   }

   public void startArUcoDetection()
   {
      undistortAndUpdateArUcoThread.start();
   }

   public void startAll()
   {
      startImagePublishing();
      startArUcoDetection();
   }

   public void stopImagePublishing()
   {
      publishDistoredColorThread.stop();
   }

   public void stopArUcoDetection()
   {
      undistortAndUpdateArUcoThread.stop();
   }

   public void stopAll()
   {
      stopImagePublishing();
      stopArUcoDetection();
   }

   public void destroy()
   {
      stopAll();

      undistortionMap1.close();
      undistortionMap2.close();
      nextGpuDistortedImage.destroy();
      imageEncoder.destroy();
      ros2DistoredImagePublisher.destroy();
      ros2Node.destroy();
   }

   public void setNextDistortedImage(RawImage distortedImage)
   {
      if (distortedImage != null && imageWidth == 0 && imageHeight == 0)
      {
         imageWidth = distortedImage.getImageWidth();
         imageHeight = distortedImage.getImageHeight();
         initializeImageUndistortion();
      }

      imagePublishLock.lock();
      updateArUcoLock.lock();
      try
      {
         if (nextGpuDistortedImage != null)
            nextGpuDistortedImage.destroy();
         nextGpuDistortedImage = distortedImage;
         newImageAvailable.signal();
         updateArUcoCondition.signal();
      }
      finally
      {
         imagePublishLock.unlock();
         updateArUcoLock.unlock();
      }
   }
}
