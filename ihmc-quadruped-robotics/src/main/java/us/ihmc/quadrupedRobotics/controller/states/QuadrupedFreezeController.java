package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class QuadrupedFreezeController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedFreezeController.class.getSimpleName());

   private final DoubleParameter freezeJointStiffness = new DoubleParameter("freezeJointStiffness", registry, 500.0);
   private final DoubleParameter freezeJointDamping = new DoubleParameter("freezeJointDamping", registry, 25.0);

   // Yo variables
   private final YoBoolean yoUseForceFeedbackControl;

   private final QuadrupedFeetManager feetManager;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedControllerToolbox controllerToolbox;

   private final JointDesiredOutputList jointDesiredOutputList;
   private final ArrayList<YoDouble> desiredFreezePositions = new ArrayList<>();
   private final ArrayList<OneDoFJoint> joints = new ArrayList<>();

   public QuadrupedFreezeController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                    QuadrupedControlMode controlMode, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();

      // Yo variables
      yoUseForceFeedbackControl = new YoBoolean("useForceFeedbackControl", registry);
      yoUseForceFeedbackControl.set(controlMode == QuadrupedControlMode.FORCE);

      feetManager = controlManagerFactory.getOrCreateFeetManager();
      fullRobotModel = controllerToolbox.getRuntimeEnvironment().getFullRobotModel();

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         if (fullRobotModel.getNameForOneDoFJoint(joint).getRole() == JointRole.LEG)
         {
            joints.add(joint);
            desiredFreezePositions.add(new YoDouble(joint.getName() + "FreezePosition", registry));
         }
      }



      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.update();

      // Initialize sole position controller
      feetManager.requestHoldAll();

      // Initialize force feedback
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         double desiredPosition = jointDesiredOutput.hasDesiredPosition() ? jointDesiredOutput.getDesiredPosition() : joint.getQ();
         desiredFreezePositions.get(i).set(desiredPosition);
      }

      jointDesiredOutputList.clear();
   }

   @Override
   public void doAction(double timeInState)
   {
      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      feetManager.compute();

      // Initialize force feedback
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = joints.get(i);
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint);
         jointDesiredOutput.clear();
         jointDesiredOutput.setStiffness(freezeJointStiffness.getValue());
         jointDesiredOutput.setDamping(freezeJointDamping.getValue());
         jointDesiredOutput.setDesiredPosition(desiredFreezePositions.get(i).getDoubleValue());

         if (yoUseForceFeedbackControl.getBooleanValue())
            jointDesiredOutput.setControlMode(JointDesiredControlMode.EFFORT);
         else
            jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
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
      yoUseForceFeedbackControl.set(true);
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
         if (jointName.getRole().equals(JointRole.LEG))
         {
            jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.EFFORT);
         }
      }
   }
}
