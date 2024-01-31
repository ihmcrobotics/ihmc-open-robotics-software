package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SmoothTransitionControllerState extends HighLevelControllerState
{
   public static boolean REDUCE_YOVARIABLES = false;

   private final BooleanProvider enableTimeBasedTransition;
   private final DoubleProvider standTransitionDuration;
   private final YoDouble standTransitionRatioCurrentValue;
   private final YoPolynomial transitionRatioTrajectory;

   private final PairList<OneDoFJointBasics, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final HighLevelControllerState initialControllerState;
   private final HighLevelControllerState finalControllerState;
   private final CommandInputManager commandInputManager;

   public SmoothTransitionControllerState(String namePrefix,
                                          HighLevelControllerName controllerState,
                                          HighLevelControllerState initialControllerState,
                                          HighLevelControllerState finalControllerState,
                                          OneDoFJointBasics[] controlledJoints,
                                          HighLevelControllerParameters highLevelControllerParameters,
                                          CommandInputManager commandInputManager)
   {
      super(namePrefix, controllerState, controlledJoints);

      this.initialControllerState = initialControllerState;
      this.finalControllerState = finalControllerState;
      this.commandInputManager = commandInputManager;

      enableTimeBasedTransition = new BooleanParameter(namePrefix + "EnableTimeBasedTransition",
                                                       "When true, the ramp up follows a linear time-based trajectory, when false, the user has to ramp up manually TransitionRatioCurrentValue through SCS.",
                                                       registry,
                                                       true);
      standTransitionDuration = new DoubleParameter(namePrefix + "TransitionDuration", registry, highLevelControllerParameters.getTimeInStandTransition());
      standTransitionRatioCurrentValue = new YoDouble(namePrefix + "TransitionRatioCurrentValue", registry);
      transitionRatioTrajectory = new YoPolynomial(namePrefix + "TransitionRatioTrajectory", 2, registry);

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      YoRegistry registryForBlenders = REDUCE_YOVARIABLES ? null : registry;

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         JointControlBlender jointControlBlender = new JointControlBlender("_StandTransition", controlledJoint, registryForBlenders);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);
      }
   }

   @Override
   public void onEntry()
   {
      finalControllerState.onEntry();
      transitionRatioTrajectory.setLinear(0.0, standTransitionDuration.getValue(), 0.0, 1.0);
      standTransitionRatioCurrentValue.set(0.0);

      commandInputManager.clearAllCommands();
      commandInputManager.setEnabled(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      initialControllerState.doAction(timeInState);
      finalControllerState.doAction(timeInState);

      double gainRatio;

      if (enableTimeBasedTransition.getValue())
      {
         double timeInBlending = MathTools.clamp(timeInState, 0.0, standTransitionDuration.getValue());
         transitionRatioTrajectory.compute(timeInBlending);
         gainRatio = transitionRatioTrajectory.getValue();
         standTransitionRatioCurrentValue.set(gainRatio);
      }
      else
      {
         gainRatio = standTransitionRatioCurrentValue.getValue();
      }

      if (Double.isNaN(gainRatio))
         gainRatio = 0.0;
      else
         gainRatio = MathTools.clamp(gainRatio, 0.0, 1.0);

      JointDesiredOutputListReadOnly standReadyJointCommand = initialControllerState.getOutputForLowLevelController();
      JointDesiredOutputListReadOnly walkingJointCommand = finalControllerState.getOutputForLowLevelController();

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointCommandBlenders.get(jointIndex).getLeft();
         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();

         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData,
                                                          standReadyJointCommand.getJointDesiredOutput(joint),
                                                          walkingJointCommand.getJointDesiredOutput(joint),
                                                          gainRatio);
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      initialControllerState.onExit(timeInState);
      commandInputManager.setEnabled(true);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (enableTimeBasedTransition.getValue())
         return timeInState > standTransitionDuration.getValue();
      else
         return standTransitionRatioCurrentValue.getValue() >= 1.0;
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
