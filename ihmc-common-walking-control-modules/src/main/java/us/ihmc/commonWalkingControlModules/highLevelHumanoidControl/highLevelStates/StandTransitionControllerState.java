package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandTransitionControllerState extends HighLevelControllerState
{
   private static final HighLevelController controllerState = HighLevelController.STAND_TRANSITION_STATE;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble standTransitionDuration = new YoDouble("standTransitionDuration", registry);
   private final YoPolynomial walkingControlRatioTrajectory = new YoPolynomial("walkingControlRatioTrajectory", 2, registry);

   private final PairList<OneDoFJoint, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final HighLevelControllerState standReadyControllerState;
   private final HighLevelControllerState walkingControllerState;

   public StandTransitionControllerState(HighLevelControllerState standReadyControllerState, HighLevelControllerState walkingControllerState,
                                         HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState);

      this.standReadyControllerState = standReadyControllerState;
      this.walkingControllerState = walkingControllerState;
      this.standTransitionDuration.set(highLevelControllerParameters.getTimeInStandTransition());

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         JointControlBlender jointControlBlender = new JointControlBlender("_StandTransition", controlledJoint, registry);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);

         String jointName = controlledJoint.getName();
         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(jointName, controllerState);
         double jointStiffness = highLevelControllerParameters.getLowLevelJointStiffness(jointName);
         double jointDamping = highLevelControllerParameters.getLowLevelJointDamping(jointName);

         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(controlledJoint);
         lowLevelJointData.setControlMode(jointControlMode);
         lowLevelJointData.setKp(jointStiffness);
         lowLevelJointData.setKd(jointDamping);
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

   @Override
   public void warmup(int iterations)
   {
   }
}
