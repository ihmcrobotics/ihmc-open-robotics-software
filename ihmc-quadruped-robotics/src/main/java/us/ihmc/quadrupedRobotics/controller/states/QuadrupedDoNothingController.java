package us.ihmc.quadrupedRobotics.controller.states;

import java.util.ArrayList;

import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A controller that does nothing, but signifies that the robot is ready to transition to stand prep
 */
public class QuadrupedDoNothingController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointDesiredOutputList jointDesiredOutputList;
   private final ArrayList<YoDouble> desiredDoNothingTorques = new ArrayList<>();
   private final ArrayList<OneDoFJoint> legJoints = new ArrayList<>();

   private final QuadrupedControllerToolbox controllerToolbox;

   public QuadrupedDoNothingController(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      FullQuadrupedRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         legJoints.add(joint);
         desiredDoNothingTorques.add(new YoDouble(joint.getName() + "DoNothingTorque", registry));
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
         if (controllerToolbox.isPositionControlled())
            jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
         else
            jointDesiredOutput.setControlMode(JointDesiredControlMode.EFFORT);
         jointDesiredOutput.setStiffness(0.0);
         jointDesiredOutput.setDamping(0.0);
         jointDesiredOutput.setDesiredTorque(0.0);
      }

   }

   @Override
   public void doAction(double timeInState)
   {
      for (int i = 0; i < legJoints.size(); i++)
      {
         OneDoFJoint joint = legJoints.get(i);
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setDesiredTorque(desiredDoNothingTorques.get(i).getDoubleValue());
      }
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }
}

