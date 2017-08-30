package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class NewStandReadyControllerState extends NewHighLevelControllerState
{
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);

   private final OneDoFJoint[] controlledJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final StandPrepSetpoints standPrepSetpoints;

   public NewStandReadyControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepSetpoints standPrepSetpoints)
   {
      super(NewHighLevelControllerStates.STAND_READY_STATE);

      this.standPrepSetpoints = standPrepSetpoints;

      controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doAction()
   {
      for (int jointIndex = 0; jointIndex < controlledJoints.length; jointIndex++)
      {
         OneDoFJoint joint = controlledJoints[jointIndex];
         double qDesired = standPrepSetpoints.get(joint.getName());
         double qdDesired = 0.0;

         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(joint, qDesired);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(joint, qdDesired);
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
      return null;
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

}
