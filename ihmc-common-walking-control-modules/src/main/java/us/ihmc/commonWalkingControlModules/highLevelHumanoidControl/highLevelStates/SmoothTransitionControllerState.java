package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoDouble;

public class SmoothTransitionControllerState extends HighLevelControllerState
{
   private final DoubleProvider standTransitionDuration;
   private final YoDouble standTransitionRatioCurrentValue;
   private final YoPolynomial transitionRatioTrajectory;

   private final PairList<OneDoFJointBasics, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final HighLevelControllerState initialControllerState;
   private final HighLevelControllerState finalControllerState;

   public SmoothTransitionControllerState(String namePrefix, HighLevelControllerName controllerState, HighLevelControllerState initialControllerState,
                                          HighLevelControllerState finalControllerState, OneDoFJointBasics[] controlledJoints,
                                          HighLevelControllerParameters highLevelControllerParameters)
   {
      super(namePrefix, controllerState, highLevelControllerParameters, controlledJoints);

      this.initialControllerState = initialControllerState;
      this.finalControllerState = finalControllerState;

      standTransitionDuration = new DoubleParameter(namePrefix + "TransitionDuration", registry, highLevelControllerParameters.getTimeInStandTransition());
      standTransitionRatioCurrentValue = new YoDouble(namePrefix + "TransitionRatioCurrentValue", registry);
      transitionRatioTrajectory = new YoPolynomial(namePrefix + "TransitionRatioTrajectory", 2, registry);

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         JointControlBlender jointControlBlender = new JointControlBlender("_StandTransition", controlledJoint, registry);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);
      }
   }

   @Override
   public void onEntry()
   {
      finalControllerState.onEntry();

      transitionRatioTrajectory.setLinear(0.0, standTransitionDuration.getValue(), 0.0, 1.0);
   }

   @Override
   public void doAction(double timeInState)
   {
      initialControllerState.doAction(timeInState);
      finalControllerState.doAction(timeInState);

      double timeInBlending = MathTools.clamp(timeInState, 0.0, standTransitionDuration.getValue());
      transitionRatioTrajectory.compute(timeInBlending);
      double gainRatio = transitionRatioTrajectory.getPosition();
      standTransitionRatioCurrentValue.set(gainRatio);

      JointDesiredOutputListReadOnly standReadyJointCommand = initialControllerState.getOutputForLowLevelController();
      JointDesiredOutputListReadOnly walkingJointCommand = finalControllerState.getOutputForLowLevelController();

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointCommandBlenders.get(jointIndex).getLeft();
         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();

         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData, standReadyJointCommand.getJointDesiredOutput(joint),
                                                          walkingJointCommand.getJointDesiredOutput(joint), gainRatio);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit(double timeInState)
   {
      initialControllerState.onExit(timeInState);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > standTransitionDuration.getValue();
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
