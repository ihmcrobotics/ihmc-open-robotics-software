package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

public class LocateFiducialBehavior extends AbstractBehavior
{
   public LocateFiducialBehavior(CommunicationBridgeInterface communicationBridge, GoalDetectorBehaviorService fiducialDetectorBehaviorService)
   {
      super(fiducialDetectorBehaviorService.getClass().getSimpleName(), communicationBridge);

      addBehaviorService(fiducialDetectorBehaviorService);
   }

   @Override
   public void doControl()
   {
      
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void onBehaviorEntered()
   {
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }
}
