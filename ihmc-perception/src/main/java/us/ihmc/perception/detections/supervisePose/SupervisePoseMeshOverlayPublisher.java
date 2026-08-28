package us.ihmc.perception.detections.supervisePose;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.ImageMessage;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.sensors.CameraIntrinsics;

import java.util.Collection;

public class SupervisePoseMeshOverlayPublisher implements AutoCloseable
{
   /**
    * Combined camera-driven overlay:
    *
    * /ihmc/supervisepose/overlayed_image
    */
   private final ROS2Publisher<ImageMessage> meshOverlayPublisher;

   public SupervisePoseMeshOverlayPublisher(ROS2Node ros2Node)
   {
      meshOverlayPublisher = ros2Node.createPublisher(SupervisePoseAPI.SUPERVISE_POSE_OVERLAY_IMAGE);
   }

   /**
    * Called from the camera loop.
    *
    * Draws all currently tracked objects using their latest available poses.
    */
   public void publish(RawImage colorImage, Collection<SupervisePoseCommunicator> communicators)
   {
      if (colorImage == null
          || communicators == null
          || communicators.isEmpty())
      {
         return;
      }

      boolean hasTrackedCommunicator = false;

      for (SupervisePoseCommunicator communicator : communicators)
      {
         if (communicator.isEnabled()
             && communicator.getState()
                == SupervisePoseCommunicator.State.TRACKING)
         {
            hasTrackedCommunicator = true;
            break;
         }
      }

      if (!hasTrackedCommunicator)
         return;

      RawImage imageSnapshot = colorImage.get();

      if (imageSnapshot == null)
         return;

      Mat overlayMat = null;
      BytePointer encodedImage = null;

      try
      {
         Mat sourceImage = imageSnapshot.getCpuImageMat();

         if (sourceImage == null || sourceImage.isNull())
            return;

         overlayMat = sourceImage.clone();

         CameraIntrinsics cameraIntrinsics = imageSnapshot.getIntrinsicsCopy();

         for (SupervisePoseCommunicator communicator : communicators)
         {
            if (!communicator.isEnabled()
                || communicator.getState()
                   != SupervisePoseCommunicator.State.TRACKING)
            {
               continue;
            }

            communicator.renderMeshOverlay(overlayMat, cameraIntrinsics);
         }

         encodedImage = new BytePointer();

         boolean encoded = opencv_imgcodecs.imencode(".jpg", overlayMat, encodedImage);

         if (!encoded)
         {
            LogTools.error("Failed to encode combined SupervisePose mesh overlay");
            return;
         }

         ImageMessage imageMessage = new ImageMessage();

         PerceptionMessageTools.packImageMessage(imageSnapshot, encodedImage, CompressionType.JPEG, imageMessage);

         meshOverlayPublisher.publish(imageMessage);
      }
      catch (Throwable throwable)
      {
         LogTools.error("Failed to publish combined SupervisePose mesh overlay: " + throwable.getMessage());
      }
      finally
      {
         if (encodedImage != null)
            encodedImage.deallocate();

         if (overlayMat != null)
            overlayMat.close();

         imageSnapshot.release();
      }
   }

   @Override
   public void close()
   {
      meshOverlayPublisher.remove();
   }
}