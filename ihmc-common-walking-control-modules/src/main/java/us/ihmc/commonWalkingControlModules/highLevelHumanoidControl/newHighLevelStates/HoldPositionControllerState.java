package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HoldPositionControllerState extends NewHighLevelControllerState
{
   private final YoVariableRegistry registry;

   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private final LowLevelOneDoFJointDesiredDataHolderList highLevelControllerOutput;
   protected final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, YoDouble> jointSetpoints = new PairList<>();

   public HoldPositionControllerState(NewHighLevelControllerStates stateEnum, HighLevelHumanoidControllerToolbox controllerToolbox,
                                      LowLevelOneDoFJointDesiredDataHolderList highLevelControllerOutput)
   {
      super(stateEnum);

      this.controllerToolbox = controllerToolbox;
      this.highLevelControllerOutput = highLevelControllerOutput;
      String nameSuffix = stateEnum.name();
      registry = new YoVariableRegistry(nameSuffix + getClass().getSimpleName());
      nameSuffix = "_" + nameSuffix;

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoDouble freezePosition = new YoDouble(jointName + nameSuffix + "_qDesired", registry);
         freezePosition.setToNaN();

         jointSetpoints.add(controlledJoint, freezePosition);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJoint joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble setpoint = jointSetpoints.get(jointIndex).getRight();
         LowLevelJointData lowLevelJointData = highLevelControllerOutput.getLowLevelJointData(joint);
         setpoint.set(lowLevelJointData.getDesiredPosition());
      }
   }

   @Override
   public void doAction()
   {
      controllerToolbox.update();
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJoint joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble desiredPosition = jointSetpoints.get(jointIndex).getRight();

         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(0.0);
         lowLevelJointData.setDesiredAcceleration(0.0);
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

   @Override
   public void warmup(int iterations)
   {}
}
