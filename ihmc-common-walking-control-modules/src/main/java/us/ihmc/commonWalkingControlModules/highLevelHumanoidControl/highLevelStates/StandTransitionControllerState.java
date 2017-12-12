package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandTransitionControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.STAND_TRANSITION_STATE;

   private final YoDouble standTransitionDuration = new YoDouble("standTransitionDuration", registry);
   private final YoDouble standTransitionGainRatio = new YoDouble("standTransitionGainRatio", registry);
   private final YoPolynomial walkingControlRatioTrajectory = new YoPolynomial("walkingControlRatioTrajectory", 2, registry);

   private final PairList<OneDoFJoint, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final HighLevelControllerState standReadyControllerState;
   private final HighLevelControllerState walkingControllerState;

   public StandTransitionControllerState(HighLevelControllerState standReadyControllerState, HighLevelControllerState walkingControllerState,
                                         HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox);

      this.standReadyControllerState = standReadyControllerState;
      this.walkingControllerState = walkingControllerState;
      this.standTransitionDuration.set(highLevelControllerParameters.getTimeInStandTransition());

      OneDoFJoint[] controlledJoints = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         JointControlBlender jointControlBlender = new JointControlBlender("_StandTransition", controlledJoint, registry);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);
      }
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

      double timeInBlending = MathTools.clamp(getTimeInCurrentState(), 0.0, standTransitionDuration.getDoubleValue());
      walkingControlRatioTrajectory.compute(timeInBlending);
      double gainRatio = walkingControlRatioTrajectory.getPosition();
      standTransitionGainRatio.set(gainRatio);

      JointDesiredOutputListReadOnly standReadyJointCommand = standReadyControllerState.getOutputForLowLevelController();
      JointDesiredOutputListReadOnly walkingJointCommand = walkingControllerState.getOutputForLowLevelController();

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJoint joint = jointCommandBlenders.get(jointIndex).getLeft();
         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         JointDesiredOutput lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();

         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData, standReadyJointCommand.getJointDesiredOutput(joint),
                                                          walkingJointCommand.getJointDesiredOutput(joint), gainRatio);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
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
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
