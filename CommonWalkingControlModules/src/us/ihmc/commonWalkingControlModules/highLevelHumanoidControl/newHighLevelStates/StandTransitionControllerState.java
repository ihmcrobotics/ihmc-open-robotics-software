package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.JointControlBlender;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandTransitionControllerState extends NewHighLevelControllerState
{
   private static final NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.STAND_TRANSITION_STATE;
   private static final double TIME_TO_RAMP_UP_CONTROL = 0.7;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble standTransitionDuration = new YoDouble("standTransitionDuration", registry);
   private final YoPolynomial walkingControlRatioTrajectory = new YoPolynomial("walkingControlRatioTrajectory", 2, registry);

   private final PairList<OneDoFJoint, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final NewHighLevelControllerState standReadyControllerState;
   private final NewHighLevelControllerState walkingControllerState;

   public StandTransitionControllerState(NewHighLevelControllerState standReadyControllerState, NewHighLevelControllerState walkingControllerState,
                                         HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      this(standReadyControllerState, walkingControllerState, controllerToolbox, highLevelControllerParameters, TIME_TO_RAMP_UP_CONTROL);
   }

   public StandTransitionControllerState(NewHighLevelControllerState standReadyControllerState, NewHighLevelControllerState walkingControllerState,
                                         HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                         double standTransitionDuration)
   {
      super(controllerState);

      this.standReadyControllerState = standReadyControllerState;
      this.walkingControllerState = walkingControllerState;
      this.standTransitionDuration.set(standTransitionDuration);

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         JointControlBlender jointControlBlender = new JointControlBlender("_StandTransition", controlledJoint, registry);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);

         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(controlledJoint.getName(), controllerState);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(controlledJoint, jointControlMode);
      }
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      walkingControllerState.doTransitionIntoAction();

      walkingControlRatioTrajectory.setLinear(0.0, standTransitionDuration.getDoubleValue(), 0.0, 1.0);
   }

   @Override
   public void doAction()
   {
      standReadyControllerState.doAction();
      walkingControllerState.doAction();

      walkingControlRatioTrajectory.compute(getTimeInCurrentState());
      double gainRatio = walkingControlRatioTrajectory.getPosition();

      LowLevelOneDoFJointDesiredDataHolderReadOnly standReadyJointCommand = standReadyControllerState.getOutputForLowLevelController();
      LowLevelOneDoFJointDesiredDataHolderReadOnly walkingJointCommand = walkingControllerState.getOutputForLowLevelController();

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJoint joint = jointCommandBlenders.get(jointIndex).getLeft();
         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);

         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData, standReadyJointCommand.getLowLevelJointData(joint),
                                                          walkingJointCommand.getLowLevelJointData(joint), gainRatio);
      }
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
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

}
