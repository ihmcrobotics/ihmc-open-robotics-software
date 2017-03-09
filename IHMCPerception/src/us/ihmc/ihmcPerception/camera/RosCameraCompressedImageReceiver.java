package us.ihmc.ihmcPerception.camera;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;

import java.awt.image.BufferedImage;

public class RosCameraCompressedImageReceiver extends RosCameraReceiver
{
   public RosCameraCompressedImageReceiver(DRCRobotCameraParameters cameraParameters, RosMainNode rosMainNode, CameraLogger logger, CameraDataReceiver cameraDataReceiver)
   {
      super(cameraParameters, rosMainNode, logger, cameraDataReceiver);
   }

   @Override
   protected void createImageSubscriber(final RobotSide robotSide, final CameraLogger logger, final CameraDataReceiver cameraDataReceiver,
         final RosCameraInfoSubscriber imageInfoSubscriber, RosMainNode rosMainNode, DRCRobotCameraParameters cameraParameters)
   {
      RosCompressedImageSubscriber imageSubscriberSubscriber = new RosCompressedImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }
            IntrinsicParameters intrinsicParameters = imageInfoSubscriber.getIntrinisicParameters();
            if (DEBUG)
            {
               PrintTools.debug(this, "Sending intrinsicParameters");
               intrinsicParameters.print();
            }
            cameraDataReceiver.updateImage(VideoSource.getMultisenseSourceFromRobotSide(robotSide), image, timeStamp, intrinsicParameters);

         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);
   }
}
