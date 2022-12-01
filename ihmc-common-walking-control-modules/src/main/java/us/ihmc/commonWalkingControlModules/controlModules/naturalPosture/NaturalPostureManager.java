package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
//import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
//import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
//import us.ihmc.robotics.stateMachine.core.StateMachine;
//import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
//import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
//import us.ihmc.yoVariables.variable.YoDouble;
//import us.ihmc.yoVariables.variable.YoEnum;

public class NaturalPostureManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ControllerNaturalPostureManager walkingManager;
//   private final UserNaturalPostureManager userManager;   
   
   private HumanoidRobotNaturalPosture robotNaturalPosture;
   private final JointLimitEnforcementMethodCommand jointLimitEnforcementMethodCommand = new JointLimitEnforcementMethodCommand();

   private final ExecutionTimer naturalPostureTimer;
   
//   private final StateMachine<NaturalPostureControlMode, NaturalPostureControlState> stateMachine;
//   private final YoEnum<NaturalPostureControlMode> requestedState;
   
   public HumanoidRobotNaturalPosture getRobotNaturalPosture() {
      return robotNaturalPosture;
   }
   
   public NaturalPostureManager(HumanoidRobotNaturalPosture naturalPostureMeasurement,
                                NaturalPostureParameters naturalPostureParameters,
                                PID3DGainsReadOnly gains,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
//      YoDouble yoTime = controllerToolbox.getYoTime();
//      String namePrefix = getClass().getSimpleName();
//      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, NaturalPostureControlMode.class, true);
      
      robotNaturalPosture = naturalPostureMeasurement;
      
//      stateMachine = setupStateMachine(namePrefix, yoTime);
      
      walkingManager = new ControllerNaturalPostureManager(robotNaturalPosture, gains, controllerToolbox, registry);
      naturalPostureTimer = new ExecutionTimer("naturalPostureTimer", registry);


      OneDoFJointBasics[] allOneDoFjoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);

      String[] jointNamesRestrictiveLimits = naturalPostureParameters.getJointsWithRestrictiveLimits();
      OneDoFJointBasics[] jointsWithRestrictiveLimit = MultiBodySystemTools.filterJoints(ScrewTools.findJointsWithNames(allOneDoFjoints,
                                                                                                                        jointNamesRestrictiveLimits),
                                                                                         OneDoFJointBasics.class);
      for (OneDoFJointBasics joint : jointsWithRestrictiveLimit)
      {
         JointLimitParameters limitParameters = naturalPostureParameters.getJointLimitParametersForJointsWithRestrictiveLimits(joint.getName());
         if (limitParameters == null)
            throw new RuntimeException("Must define joint limit parameters for joint " + joint.getName() + " if using joints with restrictive limits.");
         jointLimitEnforcementMethodCommand.addLimitEnforcementMethod(joint, JointLimitEnforcement.RESTRICTIVE, limitParameters);
      }

   }
   
//   private StateMachine<NaturalPostureControlMode, NaturalPostureControlState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
//   {
//      StateMachineFactory<NaturalPostureControlMode, NaturalPostureControlState> factory = new StateMachineFactory<>(NaturalPostureControlMode.class);
//      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);
//
//      factory.addState(NaturalPostureControlMode.WALKING_CONTROLLER, walkingManager);
////      factory.addState(NaturalPostureControlMode.USER, userManager);
//
//      for (NaturalPostureControlMode from : NaturalPostureControlMode.values())
//      {
//         factory.addRequestedTransition(from, requestedState);
//         factory.addRequestedTransition(from, from, requestedState);
//      }
//
//      return factory.build(NaturalPostureControlMode.WALKING_CONTROLLER);
//   }

   public InverseDynamicsCommand<?> getQPObjectiveCommand()
   {
      return walkingManager.getInverseDynamicsCommand();
   }
   
   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return walkingManager.getPelvisPrivilegedPoseCommand();
   }

   public JointLimitEnforcementMethodCommand getJointLimitEnforcementCommand()
   {

   }
   
//   public void setWeights(Vector3DReadOnly weight)
//   {
//      walkingManager.setWeights(weight);
//   }
   
   public void compute()
   {
      naturalPostureTimer.startMeasurement();
//      stateMachine.doAction();
//      stateMachine.doActionAndTransition();
      walkingManager.compute();
      naturalPostureTimer.stopMeasurement();
   }
   
   public void initialize()
   {
//      requestState(NaturalPostureControlMode.WALKING_CONTROLLER);
      robotNaturalPosture.initialize();
   }
   
//   private void requestState(NaturalPostureControlMode state)
//   {
//      if (stateMachine.getCurrentStateKey() != state)
//      {
//         requestedState.set(state);
//      }
//   }
   
//   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
//   {
//      walkingManager.setSelectionMatrix(selectionMatrix);
//   }
}