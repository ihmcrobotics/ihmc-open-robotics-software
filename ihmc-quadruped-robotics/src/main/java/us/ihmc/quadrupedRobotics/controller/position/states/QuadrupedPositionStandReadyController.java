package us.ihmc.quadrupedRobotics.controller.position.states;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to
 */
public class QuadrupedPositionStandReadyController implements QuadrupedController
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
   public ControllerEvent process()
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }
}

