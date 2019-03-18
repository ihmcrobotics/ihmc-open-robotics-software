package us.ihmc.quadrupedRobotics.controller.states;

import java.util.ArrayList;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedFreezeController implements State
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
   private final ArrayList<OneDoFJointBasics> joints = new ArrayList<>();

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

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
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

      feetManager.requestFullContact();

      controllerToolbox.updateSupportPolygon();

      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJointBasics joint = joints.get(i);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         double desiredPosition = jointDesiredOutput.hasDesiredPosition() ? jointDesiredOutput.getDesiredPosition() : joint.getQ();
         desiredFreezePositions.get(i).set(desiredPosition);
      }

      jointDesiredOutputList.clear();
   }

   @Override
   public void doAction(double timeInState)
   {
      controllerToolbox.update();

      feetManager.compute();

      controllerToolbox.updateSupportPolygon();

      // Initialize force feedback
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = joints.get(i);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint);
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
   public boolean isDone(double timeInState)
   {
      return false;
   }

   @Override
   public void onExit()
   {
      yoUseForceFeedbackControl.set(true);
      for (OneDoFJointBasics oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
         if (jointName.getRole().equals(JointRole.LEG))
         {
            jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.EFFORT);
         }
      }
   }
}
