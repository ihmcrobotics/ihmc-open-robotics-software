package us.ihmc.perception.ros1.camera;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

import java.awt.image.BufferedImage;

@Deprecated
public class RosCameraImageReceiver extends RosCameraReceiver
{
   public RosCameraImageReceiver(AvatarRobotCameraParameters cameraParameters, RosMainNode rosMainNode, CameraLogger logger, CameraDataReceiver cameraDataReceiver)
   {
      super(cameraParameters, rosMainNode, logger, cameraDataReceiver);
   }

   @Override
   protected void createImageSubscriber(final RobotSide robotSide, final CameraLogger logger, final CameraDataReceiver cameraDataReceiver,
         final RosCameraInfoSubscriber imageInfoSubscriber, RosMainNode rosMainNode, AvatarRobotCameraParameters cameraParameters)
   {
      RosImageSubscriber imageSubscriber = new RosImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }
//            CameraPinholeBrown intrinsicParameters = imageInfoSubscriber.getIntrinisicParameters();
            if (DEBUG)
            {
               PrintTools.debug(this, "Sending intrinsicParameters");
//               intrinsicParameters.print();
            }
//            cameraDataReceiver.updateImage(VideoSource.getMultisenseSourceFromRobotSide(robotSide), image, timeStamp, intrinsicParameters);
         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriber);
   }
}
