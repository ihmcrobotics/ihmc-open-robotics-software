package us.ihmc.quadrupedRobotics.controller.force.states;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class QuadrupedForceBasedFreezeController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedFreezeController.class.getSimpleName());

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);

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

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final QuadrantDependentList<VirtualWrenchCommand> virtualWrenchCommands = new QuadrantDependentList<>();

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
         VirtualWrenchCommand command = new VirtualWrenchCommand();
         command.getSelectionMatrix().set(linearSelectionMatrix);
         command.setRigidBody(fullRobotModel.getFoot(robotQuadrant));
         virtualWrenchCommands.set(robotQuadrant, command);
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
   public ControllerEvent process()
   {
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());

      controllerToolbox.update();

      feetManager.compute(taskSpaceControllerCommands.getSoleForce());
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

      inverseDynamicsCommandList.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         convertToVirtualWrenchCommand(taskSpaceControllerCommands.getSoleForce(robotQuadrant), virtualWrenchCommands.get(robotQuadrant));
         inverseDynamicsCommandList.addCommand(virtualWrenchCommands.get(robotQuadrant));
      }

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

   private void convertToVirtualWrenchCommand(FrameVector3D soleForce, VirtualWrenchCommand virtualWrenchCommand)
   {
      virtualWrenchCommand.getVirtualWrench().setToZero(ReferenceFrame.getWorldFrame());
      virtualWrenchCommand.getVirtualWrench().setLinearPart(soleForce);
   }
}
