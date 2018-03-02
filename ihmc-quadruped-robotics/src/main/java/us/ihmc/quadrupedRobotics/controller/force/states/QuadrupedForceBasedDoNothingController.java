package us.ihmc.quadrupedRobotics.controller.force.states;

import java.util.ArrayList;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to stand prep
 */
public class QuadrupedForceBasedDoNothingController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointDesiredOutputList jointDesiredOutputList;
   private final ArrayList<YoDouble> desiredDoNothingTorques = new ArrayList<>();
   private final ArrayList<OneDoFJoint> legJoints = new ArrayList<>();

   public QuadrupedForceBasedDoNothingController(QuadrupedRuntimeEnvironment environment, YoVariableRegistry parentRegistry)
   {
      FullQuadrupedRobotModel fullRobotModel = environment.getFullRobotModel();
      this.jointDesiredOutputList = environment.getJointDesiredOutputList();

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         if (fullRobotModel.getNameForOneDoFJoint(joint).getRole() == JointRole.LEG)
         {
            legJoints.add(joint);
            desiredDoNothingTorques.add(new YoDouble(joint.getName() + "DoNothingTorque", registry));
         }
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      for (int i = 0; i < legJoints.size(); i++)
      {
         OneDoFJoint joint = legJoints.get(i);
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setControlMode(JointDesiredControlMode.EFFORT);
         jointDesiredOutput.setDesiredTorque(0.0);
      }
   }

   @Override
   public ControllerEvent process()
   {
      for (int i = 0; i < legJoints.size(); i++)
      {
         OneDoFJoint joint = legJoints.get(i);
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setControlMode(JointDesiredControlMode.EFFORT);
         jointDesiredOutput.setDesiredTorque(desiredDoNothingTorques.get(i).getDoubleValue());
      }
      return null;
   }

   @Override
   public void onExit()
   {
   }
}

