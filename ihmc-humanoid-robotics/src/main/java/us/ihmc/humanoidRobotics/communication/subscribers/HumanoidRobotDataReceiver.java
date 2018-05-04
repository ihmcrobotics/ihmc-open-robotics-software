package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;

public class HumanoidRobotDataReceiver extends RobotDataReceiver  implements PacketConsumer<RobotConfigurationData>
{
   protected final HumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;

   
   public HumanoidRobotDataReceiver(FullHumanoidRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder)
   {
      super(fullRobotModel, FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), forceSensorDataHolder);
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.fullRobotModel = fullRobotModel;

   }
   
   public HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }
   

   public HumanoidReferenceFrames getUpdatedReferenceFramesCopy()
   {
      updateRobotModel();
      HumanoidReferenceFrames ret = new HumanoidReferenceFrames(fullRobotModel);
      return ret;
   }
   
   @Override
   protected void updateFrames()
   {
      referenceFrames.updateFrames();
   }
}
