package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PlanConstrainedWholeBodyPlanningBehavior extends AbstractBehavior
{
   private ConstrainedWholeBodyPlanningToolboxOutputStatus cwbPlanningToolboxOutputStatus;
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   protected final ConcurrentListeningQueue<ConstrainedWholeBodyPlanningToolboxOutputStatus> cwbPlanStatusQueue = new ConcurrentListeningQueue<ConstrainedWholeBodyPlanningToolboxOutputStatus>(10);

   public PlanConstrainedWholeBodyPlanningBehavior(CommunicationBridge communicationBridge, YoDouble yoTime)
   {
      super(communicationBridge);

      isDone.set(false);

      this.attachNetworkListeningQueue(cwbPlanStatusQueue, ConstrainedWholeBodyPlanningToolboxOutputStatus.class);
   }

   public ConstrainedWholeBodyPlanningToolboxOutputStatus getToolboxOutputStatus()
   {
      return cwbPlanningToolboxOutputStatus;
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
      packet.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
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
