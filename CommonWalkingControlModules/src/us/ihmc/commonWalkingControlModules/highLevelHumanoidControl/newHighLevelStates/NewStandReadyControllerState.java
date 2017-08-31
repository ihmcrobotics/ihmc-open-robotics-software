package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.EffortJointControlCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.PositionJointControlCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class NewStandReadyControllerState extends NewHighLevelControllerState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, ImmutableTriple<EffortJointControlCalculator, PositionJointControlCalculator, YoDouble>> jointControllers = new PairList<>();

   private final YoDouble masterGain = new YoDouble("standReadyMasterGain", registry);

   public NewStandReadyControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                       PositionControlParameters positionControlParameters)
   {
      super(NewHighLevelControllerStates.STAND_READY_STATE);

      masterGain.set(positionControlParameters.getPositionControlMasterGain());

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         EffortJointControlCalculator effortCalculator = new EffortJointControlCalculator("_StandReady", controlledJoint, controllerToolbox.getControlDT(), registry);
         PositionJointControlCalculator positionCalculator = new PositionJointControlCalculator("_StandReady", controlledJoint, registry);
         YoDouble standReadyPosition = new YoDouble(jointName + "_StandReady_qDesired", registry);

         standReadyPosition.set(standPrepSetpoints.getSetpoint(jointName));
         effortCalculator.setProportionalGain(positionControlParameters.getProportionalGain(jointName));
         effortCalculator.setDerivativeGain(positionControlParameters.getDerivativeGain(jointName));
         effortCalculator.setIntegralGain(positionControlParameters.getIntegralGain(jointName));

         ImmutableTriple<EffortJointControlCalculator, PositionJointControlCalculator, YoDouble> triple = new ImmutableTriple<>(effortCalculator, positionCalculator, standReadyPosition);
         jointControllers.add(controlledJoint, triple);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < jointControllers.size(); jointIndex++)
      {
         ImmutableTriple<EffortJointControlCalculator, PositionJointControlCalculator, YoDouble> triple = jointControllers.get(jointIndex).getRight();
         EffortJointControlCalculator effortCalculator = triple.getLeft();
         PositionJointControlCalculator positionCalculator = triple.getMiddle();

         effortCalculator.initialize();
         positionCalculator.initialize();
      }
   }

   @Override
   public void doAction()
   {
      for (int jointIndex = 0; jointIndex < jointControllers.size(); jointIndex++)
      {
         OneDoFJoint joint = jointControllers.get(jointIndex).getLeft();
         ImmutableTriple<EffortJointControlCalculator, PositionJointControlCalculator, YoDouble> triple = jointControllers.get(jointIndex).getRight();
         EffortJointControlCalculator effortCalculator = triple.getLeft();
         PositionJointControlCalculator positionCalculator = triple.getMiddle();
         YoDouble desiredPosition = triple.getRight();

         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());

         effortCalculator.computeAndUpdateJointTorque(lowLevelJointData, masterGain.getDoubleValue());
         positionCalculator.computeAndUpdateJointPosition(lowLevelJointData, masterGain.getDoubleValue());
      }

      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

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
