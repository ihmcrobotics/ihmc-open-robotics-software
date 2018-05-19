package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyTrajectoryPlanningBehavior extends AbstractBehavior
{

   
   public WholeBodyTrajectoryPlanningBehavior(String namePrefix, FullHumanoidRobotModelFactory fullRobotModelFactory, YoDouble yoTime,
                                              CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      super(namePrefix, outgoingCommunicationBridge);
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

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }
}
