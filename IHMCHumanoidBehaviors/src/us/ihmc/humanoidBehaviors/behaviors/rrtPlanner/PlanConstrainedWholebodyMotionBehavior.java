package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.RRTPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PlanConstrainedWholebodyMotionBehavior extends AbstractBehavior
{
   private RRTPlanningToolboxOutputStatus rrtPlanningToolboxOutputStatus;
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   
   protected final ConcurrentListeningQueue<RRTPlanningToolboxOutputStatus> rrtPlanStatusQueue = new ConcurrentListeningQueue<RRTPlanningToolboxOutputStatus>(10);
   
   public PlanConstrainedWholebodyMotionBehavior(CommunicationBridge communicationBridge, YoDouble yoTime)
   {
      super(communicationBridge);
      
      isDone.set(false);

      this.attachNetworkListeningQueue(rrtPlanStatusQueue, RRTPlanningToolboxOutputStatus.class);
   }

   public RRTPlanningToolboxOutputStatus getToolboxOutputStatus()
   {
      return rrtPlanningToolboxOutputStatus;
   }
   
   public WholeBodyTrajectoryMessage getWholebodyTrajectoryMessage()
   {
      return wholebodyTrajectoryMessage;
   }
   
   public void setToolbox()
   {
      
   }
   
   private void sendPackageToPlanner(Packet<?> packet)
   {
      packet.setDestination(PacketDestination.RRTPLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(packet);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorEntered()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub
      
   }

}
