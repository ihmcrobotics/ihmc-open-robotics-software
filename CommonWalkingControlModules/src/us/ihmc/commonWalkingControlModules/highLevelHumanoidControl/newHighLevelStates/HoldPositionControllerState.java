package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.JointControlCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HoldPositionControllerState extends NewHighLevelControllerState
{
   private final YoVariableRegistry registry;

   protected final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, ImmutablePair<YoDouble, JointControlCalculator>> jointControllers = new PairList<>();

   private final YoDouble masterGain;

   public HoldPositionControllerState(NewHighLevelControllerStates stateEnum, HighLevelHumanoidControllerToolbox controllerToolbox,
                                      PositionControlParameters positionControlParameters)
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
         JointControlCalculator jointControlCalculator = new JointControlCalculator(nameSuffix, controlledJoint, controllerToolbox.getControlDT(), registry);

         YoDouble freezePosition = new YoDouble(jointName + nameSuffix + "_qDesired", registry);
         freezePosition.setToNaN();

         jointControlCalculator.setProportionalGain(positionControlParameters.getProportionalGain(jointName));
         jointControlCalculator.setDerivativeGain(positionControlParameters.getDerivativeGain(jointName));
         jointControlCalculator.setIntegralGain(positionControlParameters.getIntegralGain(jointName));

         ImmutablePair<YoDouble, JointControlCalculator> data = new ImmutablePair<>(freezePosition, jointControlCalculator);
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
         ImmutablePair<YoDouble, JointControlCalculator> data = jointControllers.get(jointIndex).getRight();
         OneDoFJoint joint = jointControllers.get(jointIndex).getLeft();
         YoDouble setpoint = data.getLeft();
         setpoint.set(joint.getqDesired());

         JointControlCalculator jointControlCalculator = data.getRight();
         jointControlCalculator.initialize();
      }
   }

   @Override
   public void doAction()
   {
      for (int jointIndex = 0; jointIndex < jointControllers.size(); jointIndex++)
      {
         OneDoFJoint joint = jointControllers.get(jointIndex).getLeft();
         ImmutablePair<YoDouble, JointControlCalculator> data = jointControllers.get(jointIndex).getRight();
         JointControlCalculator jointControlCalculator = data.getRight();
         YoDouble desiredPosition = data.getLeft();

         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(0.0);
         lowLevelJointData.setDesiredAcceleration(0.0);

         jointControlCalculator.computeAndUpdateJointControl(lowLevelJointData, masterGain.getDoubleValue());
      }
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
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
