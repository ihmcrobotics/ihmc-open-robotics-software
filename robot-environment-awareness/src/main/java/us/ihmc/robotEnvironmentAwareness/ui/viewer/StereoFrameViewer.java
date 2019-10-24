package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class StereoFrameViewer extends AbstractSensorFrameViewer<StereoVisionPointCloudMessage>
{
   public StereoFrameViewer(REAUIMessager uiMessager, Topic<StereoVisionPointCloudMessage> messageState, Topic<Integer> numberOfFramesTopic)
   {
      super(uiMessager, messageState, numberOfFramesTopic);
   }

   @Override
   protected SensorFrame extractSensorFrameFromMessage(StereoVisionPointCloudMessage message)
   {
      SensorFrame sensorFrame = new SensorFrame(message.getSensorPosition(), message.getSensorOrientation(), message.getConfidence());
      return sensorFrame;
   }
}
