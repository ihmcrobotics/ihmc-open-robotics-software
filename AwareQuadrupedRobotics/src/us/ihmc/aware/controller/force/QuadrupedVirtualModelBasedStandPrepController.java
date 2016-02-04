package us.ihmc.aware.controller.force;

public class QuadrupedVirtualModelBasedStandPrepController implements QuadrupedForceController
{
   @Override
   public void onEntry()
   {

   }

   @Override
   public QuadrupedForceControllerEvent process()
   {
      // TODO: Don't return from state immediately.
      return QuadrupedForceControllerEvent.STARTING_POSE_REACHED;
   }

   @Override
   public void onExit()
   {

   }
}
