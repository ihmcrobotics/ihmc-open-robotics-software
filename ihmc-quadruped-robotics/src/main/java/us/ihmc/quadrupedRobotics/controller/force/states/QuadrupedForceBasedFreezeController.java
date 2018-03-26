package us.ihmc.quadrupedRobotics.controller.force.states;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedForceBasedFreezeController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedFreezeController.class.getSimpleName());

   private final DoubleParameter jointDampingParameter = new DoubleParameter("jointDamping", registry, 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = new DoubleParameter("jointPositionLimitDamping", registry, 10.0);
   private final DoubleParameter jointPositionLimitStiffnessParameter = new DoubleParameter("jointPositionLimitStiffness", registry, 100.0);
   
   // Yo variables
   private final YoBoolean yoUseForceFeedbackControl;

   // Feedback controller
   private final QuadrupedFeetManager feetManager;

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final JointDesiredOutputList jointDesiredOutputList;

   private final VirtualModelControlCommandList virtualModelControlCommandList = new VirtualModelControlCommandList();
   private final QuadrantDependentList<VirtualForceCommand> virtualForceCommands = new QuadrantDependentList<>();

   public QuadrupedForceBasedFreezeController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                              YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();

      // Yo variables
      yoUseForceFeedbackControl = new YoBoolean("useForceFeedbackControl", registry);
      // Feedback controller
      feetManager = controlManagerFactory.getOrCreateFeetManager();

      // Task space controller
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();
      fullRobotModel = controllerToolbox.getRuntimeEnvironment().getFullRobotModel();

      DenseMatrix64F linearSelectionMatrix = new DenseMatrix64F(3, Wrench.SIZE);
      linearSelectionMatrix.set(0, 3, 1);
      linearSelectionMatrix.set(1, 4, 1);
      linearSelectionMatrix.set(2, 5, 1);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         VirtualForceCommand command = new VirtualForceCommand();
         command.set(fullRobotModel.getBody(), fullRobotModel.getFoot(robotQuadrant));
         virtualForceCommands.set(robotQuadrant, command);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.update();

      // Initialize sole position controller
      feetManager.requestHoldAll();

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(quadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();

      // Initialize force feedback
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         if (yoUseForceFeedbackControl.getBooleanValue())
            jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.EFFORT);
         else
            jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.POSITION);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.getValue());

      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      feetManager.compute();
      feetManager.getDesiredSoleForceCommand(taskSpaceControllerCommands.getSoleForce());
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

      virtualModelControlCommandList.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FrameVector3D soleForce = taskSpaceControllerCommands.getSoleForce(robotQuadrant);
         virtualForceCommands.get(robotQuadrant).setLinearForce(soleForce.getReferenceFrame(), soleForce);
         virtualModelControlCommandList.addCommand(virtualForceCommands.get(robotQuadrant));
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
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            if (yoUseForceFeedbackControl.getBooleanValue())
               jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.EFFORT);
            else
               jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.POSITION);
         }
      }
   }
}
