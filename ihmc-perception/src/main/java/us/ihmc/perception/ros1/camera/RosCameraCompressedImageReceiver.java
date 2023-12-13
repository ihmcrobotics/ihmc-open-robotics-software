package us.ihmc.perception.ros1.camera;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;

public class RosCameraCompressedImageReceiver extends RosCameraReceiver
{
   private RosCompressedImageSubscriber imageSubscriber;
   private RosMainNode rosMainNode;

   public RosCameraCompressedImageReceiver(AvatarRobotCameraParameters cameraParameters, RosMainNode rosMainNode, CameraLogger logger,
                                           CameraDataReceiver cameraDataReceiver)
   {
      super(cameraParameters, rosMainNode, logger, cameraDataReceiver);
   }

   @Override
   protected void createImageSubscriber(final RobotSide robotSide, final CameraLogger logger, final CameraDataReceiver cameraDataReceiver,
                                        final RosCameraInfoSubscriber imageInfoSubscriber, RosMainNode rosMainNode,
                                        AvatarRobotCameraParameters cameraParameters)
   {
      this.rosMainNode = rosMainNode;
      imageSubscriber = new RosCompressedImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }
            CameraPinholeBrown intrinsicParameters = imageInfoSubscriber.getIntrinisicParameters();
            if (DEBUG)
            {
               LogTools.debug("Sending intrinsicParameters");
               intrinsicParameters.print();
            }
            cameraDataReceiver.updateImage(VideoSource.getMultisenseSourceFromRobotSide(robotSide), image, timeStamp, intrinsicParameters);
         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriber);
   }

   public void closeAndDispose()
   {
      if (imageSubscriber != null && rosMainNode != null)
      {
         rosMainNode.removeSubscriber(imageSubscriber);
         imageSubscriber = null;
      }
   }
}
