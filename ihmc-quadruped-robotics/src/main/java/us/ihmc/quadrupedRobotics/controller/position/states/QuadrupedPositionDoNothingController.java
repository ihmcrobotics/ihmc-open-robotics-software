package us.ihmc.quadrupedRobotics.controller.position.states;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to
 */
public class QuadrupedPositionDoNothingController implements QuadrupedController
{
   private final FullRobotModel fullRobotModel;
   private final JointDesiredOutputList jointDesiredOutputList;

   public QuadrupedPositionDoNothingController(QuadrupedRuntimeEnvironment environment)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.jointDesiredOutputList = environment.getJointDesiredOutputList();
   }

   @Override
   public void onEntry()
   {
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
         jointDesiredOutput.setDesiredPosition(joint.getQ());
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

