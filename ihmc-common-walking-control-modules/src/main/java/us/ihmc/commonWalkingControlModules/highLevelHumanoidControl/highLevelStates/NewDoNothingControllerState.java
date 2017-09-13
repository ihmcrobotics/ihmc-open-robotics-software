package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class NewDoNothingControllerState extends HighLevelControllerState
{
   private static final HighLevelController controllerState = HighLevelController.DO_NOTHING_BEHAVIOR;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final OneDoFJoint[] allRobotJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   public NewDoNothingControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState);

      this.controllerToolbox = controllerToolbox;
      allRobotJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(allRobotJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(allRobotJoints);

      OneDoFJoint[] oneDoFJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(joint.getName(), controllerState);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(joint, jointControlMode);
      }
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < allRobotJoints.length; i++)
      {
         allRobotJoints[i].setTau(0.0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(allRobotJoints[i], 0.0);
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      // Do nothing

   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
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
