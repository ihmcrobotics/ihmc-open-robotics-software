package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import org.apache.commons.lang3.tuple.ImmutablePair;
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

public class NewHoldPositionControllerState extends NewHighLevelControllerState
{
   private final YoVariableRegistry registry;

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, ImmutablePair<YoDouble, ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator>>> jointControllers = new PairList<>();

   private final YoDouble masterGain;

   public NewHoldPositionControllerState(NewHighLevelControllerStates stateEnum, HighLevelHumanoidControllerToolbox controllerToolbox,
                                         StandPrepParameters standPrepSetpoints, PositionControlParameters positionControlParameters)
   {
      super(stateEnum);

      String nameSuffix = stateEnum.name();
      registry = new YoVariableRegistry(nameSuffix + getClass().getSimpleName());
      masterGain = new YoDouble(nameSuffix + "MasterGain", registry);
      masterGain.set(positionControlParameters.getPositionControlMasterGain());

      nameSuffix = "_" + nameSuffix;

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         EffortJointControlCalculator effortCalculator = new EffortJointControlCalculator(nameSuffix, controlledJoint, controllerToolbox.getControlDT(), registry);

         PositionJointControlCalculator positionCalculator = new PositionJointControlCalculator(nameSuffix, controlledJoint, registry);
         YoDouble freezePosition = new YoDouble(jointName + nameSuffix + "_qDesired", registry);

         freezePosition.set(standPrepSetpoints.getSetpoint(jointName));
         effortCalculator.setProportionalGain(positionControlParameters.getProportionalGain(jointName));
         effortCalculator.setDerivativeGain(positionControlParameters.getDerivativeGain(jointName));
         effortCalculator.setIntegralGain(positionControlParameters.getIntegralGain(jointName));

         ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator> controllers = new ImmutablePair<>(effortCalculator, positionCalculator);
         ImmutablePair<YoDouble, ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator>> data = new ImmutablePair<>(freezePosition, controllers);
         jointControllers.add(controlledJoint, data);
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
         ImmutablePair<YoDouble, ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator>> data = jointControllers.get(jointIndex).getRight();
         ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator> controllerPair = data.getRight();
         YoDouble setpoint = data.getLeft();
         OneDoFJoint joint = jointControllers.get(jointIndex).getLeft();
         setpoint.set(joint.getqDesired());

         EffortJointControlCalculator effortCalculator = controllerPair.getLeft();
         PositionJointControlCalculator positionCalculator = controllerPair.getRight();

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
         ImmutablePair<YoDouble, ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator>> data = jointControllers.get(jointIndex).getRight();
         ImmutablePair<EffortJointControlCalculator, PositionJointControlCalculator> controllerPair = data.getRight();
         EffortJointControlCalculator effortCalculator = controllerPair.getLeft();
         PositionJointControlCalculator positionCalculator = controllerPair.getRight();
         YoDouble desiredPosition = data.getLeft();

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
