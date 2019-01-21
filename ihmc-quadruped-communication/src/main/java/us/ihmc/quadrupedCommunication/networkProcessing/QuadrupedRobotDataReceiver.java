package us.ihmc.quadrupedCommunication.networkProcessing;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;

public class QuadrupedRobotDataReceiver extends RobotDataReceiver  implements PacketConsumer<RobotConfigurationData>
{
   protected final QuadrupedReferenceFrames referenceFrames;
   private final FullQuadrupedRobotModel fullRobotModel;


   public QuadrupedRobotDataReceiver(FullQuadrupedRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder)
   {
      super(fullRobotModel, fullRobotModel.getOneDoFJoints(), forceSensorDataHolder);
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
      this.fullRobotModel = fullRobotModel;

   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }


   public QuadrupedReferenceFrames getUpdatedReferenceFramesCopy()
   {
      updateRobotModel();
      QuadrupedReferenceFrames ret = new QuadrupedReferenceFrames(fullRobotModel);
      return ret;
   }

   @Override
   protected void updateFrames()
   {
      referenceFrames.updateFrames();
   }
}
