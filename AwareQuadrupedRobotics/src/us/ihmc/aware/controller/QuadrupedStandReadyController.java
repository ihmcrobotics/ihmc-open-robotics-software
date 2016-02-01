package us.ihmc.aware.controller;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to
 */
public class QuadrupedStandReadyController implements QuadrupedController
{
   private final FullRobotModel fullRobotModel;

   public QuadrupedStandReadyController(QuadrupedRuntimeEnvironment environment)
   {
      this.fullRobotModel = environment.getFullRobotModel();
   }

   @Override
   public void onEntry()
   {
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joint.setUnderPositionControl(true);
         joint.setqDesired(joint.getQ());
      }
   }

   @Override
   public QuadrupedControllerEvent process()
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }
}

