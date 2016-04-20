package us.ihmc.aware.controller.position;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.aware.providers.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to
 */
public class QuadrupedPositionStandReadyController implements QuadrupedPositionController
{
   private final FullRobotModel fullRobotModel;

   public QuadrupedPositionStandReadyController(QuadrupedRuntimeEnvironment environment)
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
   public QuadrupedPositionControllerEvent process()
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }
}

