package us.ihmc.humanoidRobotics.communication.subscribers;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModelUtils;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
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
