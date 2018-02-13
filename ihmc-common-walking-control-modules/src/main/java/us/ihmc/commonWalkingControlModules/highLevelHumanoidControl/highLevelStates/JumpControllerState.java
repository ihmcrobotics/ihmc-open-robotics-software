package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import com.google.common.base.CaseFormat;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class JumpControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.JUMPING;
   private static final String namePrefix = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, controllerState.toString());
   
   private final WholeBodyControllerCore controllerCore;
   private final JumpHighLevelHumanoidController jumpController;

   private boolean setupControllerCoreForInverseDynamics = true;
   private boolean setupControllerCoreForInverseKinematics = false;
   private boolean setupControllerCoreForVirutalModelControl = false;

   public JumpControllerState(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, HighLevelControllerParameters parameters,
                              HighLevelHumanoidControllerToolbox controllerToolbox, JumpControlManagerFactory jumpingControlManagerFactory,
                              JumpControllerParameters jumpingControlParameters)
   {
      super(controllerState, parameters, controllerToolbox);

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();

      InverseDynamicsJoint[] jointsToOptimizeFor = controllerToolbox.getControlledJoints();
      OneDoFJoint[] controlledOneDofJoints = ScrewTools.filterJoints(jointsToOptimizeFor, OneDoFJoint.class);
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = jumpingControlParameters.getMomentumOptimizationSettings();

      WholeBodyControlCoreToolbox controllerCoreToolbox = new WholeBodyControlCoreToolbox(namePrefix, controllerToolbox.getControlDT(), controllerToolbox.getGravityZ(),
                                                                                          rootJoint, jointsToOptimizeFor,
                                                                                          controllerToolbox.getCenterOfMassFrame(),
                                                                                          controllerCoreOptimizationSettings,
                                                                                          controllerToolbox.getYoGraphicsListRegistry(), registry);
      if (setupControllerCoreForInverseDynamics)
         controllerCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());
      if (setupControllerCoreForInverseKinematics)
         controllerCoreToolbox.setupForInverseKinematicsSolver();
      if (setupControllerCoreForVirutalModelControl)
      {
         RigidBody[] controlledBodies = {fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.LEFT), fullRobotModel.getFoot(RobotSide.RIGHT)};
         controllerCoreToolbox.setupForVirtualModelControlSolver(fullRobotModel.getPelvis(), controlledBodies,
                                                                 controllerCoreToolbox.getContactablePlaneBodies());
      }

      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(controlledOneDofJoints);
      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, jumpingControlManagerFactory.createFeedbackControlTemplate(),
                                                   lowLevelControllerOutput, registry);

      jumpController = new JumpHighLevelHumanoidController(commandInputManager, statusOutputManager, controllerCoreToolbox, controllerToolbox, jumpingControlParameters,
                                                           jumpingControlManagerFactory);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public void doAction()
   {
      jumpController.doAction();
      ControllerCoreCommand controllerCoreCommand = jumpController.getControllerCoreCommand();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      
      JointDesiredOutputList stateSpecificJointSettings = getStateSpecificJointSettings();
      JointAccelerationIntegrationCommand accelerationIntegrationCommand = getAccelerationIntegrationCommand();
      controllerCoreCommand.addInverseDynamicsCommand(accelerationIntegrationCommand);
      controllerCoreCommand.completeLowLevelJointData(stateSpecificJointSettings);

      //controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      //controllerCoreTimer.stopMeasurement();
   }
   
   public void initialize()
   {
      controllerCore.initialize();
      jumpController.initialize();
   }
 
   @Override
   public void doTransitionIntoAction()
   {
      initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {

   }

}
