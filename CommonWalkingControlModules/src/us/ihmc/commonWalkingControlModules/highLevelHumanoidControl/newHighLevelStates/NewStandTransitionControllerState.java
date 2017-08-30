package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import javax.naming.ldap.Control;

public class NewStandTransitionControllerState extends NewHighLevelControllerState
{
   private static final double TIME_TO_RAMP_UP_CONTROL = 0.7;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble standTransitionDuration = new YoDouble("standTransitionDuration", registry);
   private final YoPolynomial gainRatioTrajectory = new YoPolynomial("gainRatioTrajectory", 2, registry);

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);

   private final OneDoFJoint[] controlledJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final NewStandReadyControllerState standReadyControllerState;
   private final NewWalkingControllerState walkingControllerState;

   public NewStandTransitionControllerState(NewStandReadyControllerState standReadyControllerState, NewWalkingControllerState walkingControllerState,
                                            HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this(standReadyControllerState, walkingControllerState, controllerToolbox, TIME_TO_RAMP_UP_CONTROL);
   }

   public NewStandTransitionControllerState(NewStandReadyControllerState standReadyControllerState, NewWalkingControllerState walkingControllerState,
                                            HighLevelHumanoidControllerToolbox controllerToolbox, double standTransitionDuration)
   {
      super(NewHighLevelControllerStates.STAND_TRANSITION_STATE);

      this.standReadyControllerState = standReadyControllerState;
      this.walkingControllerState = walkingControllerState;
      this.standTransitionDuration.set(standTransitionDuration);

      controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      walkingControllerState.doTransitionIntoAction();

      gainRatioTrajectory.setLinear(0.0, standTransitionDuration.getDoubleValue(), 0.0, 1.0);
   }

   @Override
   public void doAction()
   {
      standReadyControllerState.doAction();
      walkingControllerState.doAction();

      gainRatioTrajectory.compute(getTimeInCurrentState());
      double gainRatio = gainRatioTrajectory.getPosition();

      ControllerCoreCommand standReadyCommand = standReadyControllerState.getControllerCoreCommand();
      ControllerCoreCommand walkingCommand = walkingControllerState.getControllerCoreCommand();

      for (int jointIndex = 0; jointIndex < controlledJoints.length; jointIndex++)
      {
          // TODO
      }

      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      standReadyControllerState.doTransitionOutOfAction();
   }

   @Override
   public boolean isDone()
   {
      return getTimeInCurrentState() > standTransitionDuration.getDoubleValue();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

}
