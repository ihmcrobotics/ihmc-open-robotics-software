package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
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

public class QuadrupedFreezeController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedFreezeController.class.getSimpleName());

   private final DoubleParameter freezeJointStiffness = new DoubleParameter("freezeJointStiffness", registry, 25.0);
   private final DoubleParameter freezeJointDamping = new DoubleParameter("freezeJointDamping", registry, 15.0);

   // Yo variables
   private final YoBoolean yoUseForceFeedbackControl;

   private final QuadrupedFeetManager feetManager;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final JointDesiredOutputList jointDesiredOutputList;

   public QuadrupedFreezeController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                    YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();

      // Yo variables
      yoUseForceFeedbackControl = new YoBoolean("useForceFeedbackControl", registry);
      yoUseForceFeedbackControl.set(true);

      feetManager = controlManagerFactory.getOrCreateFeetManager();
      fullRobotModel = controllerToolbox.getRuntimeEnvironment().getFullRobotModel();

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.update();

      // Initialize sole position controller
      feetManager.requestHoldAll();

      // Initialize task space controller
//      taskSpaceControllerSettings.initialize();
//      for (RobotQuadrant quadrant : RobotQuadrant.values)
//      {
//         taskSpaceControllerSettings.setContactState(quadrant, ContactState.NO_CONTACT);
//      }
//      taskSpaceController.reset();

      // Initialize force feedback
      jointDesiredOutputList.clear();
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint);
         jointDesiredOutput.setStiffness(freezeJointStiffness.getValue());
         jointDesiredOutput.setDamping(freezeJointDamping.getValue());

         if (yoUseForceFeedbackControl.getBooleanValue())
            jointDesiredOutput.setControlMode(JointDesiredControlMode.EFFORT);
         else
            jointDesiredOutput.setControlMode(JointDesiredControlMode.POSITION);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      feetManager.compute();
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
