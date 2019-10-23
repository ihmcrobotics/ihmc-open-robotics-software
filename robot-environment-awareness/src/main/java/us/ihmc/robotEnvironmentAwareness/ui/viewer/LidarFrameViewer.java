package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import controller_msgs.msg.dds.LidarScanMessage;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class LidarFrameViewer extends AbstractSensorFrameViewer<LidarScanMessage>
{
   public LidarFrameViewer(REAUIMessager uiMessager, Topic<LidarScanMessage> messageState)
   {
      super(uiMessager, messageState, null);
   }

   @Override
   protected SensorFrame extractSensorFrameFromMessage(LidarScanMessage message)
   {
      SensorFrame sensorFrame = new SensorFrame(message.getLidarPosition(), message.getLidarOrientation(), 1.0);
      return sensorFrame;
   }
}
