package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class NewDoNothingControllerState extends NewHighLevelControllerState
{
   private static final NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.DO_NOTHING_STATE;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();

   private final OneDoFJoint[] allRobotJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   public NewDoNothingControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState);

      this.bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();
      this.controllerToolbox = controllerToolbox;
      allRobotJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (RobotSide robotSide : RobotSide.values)
      {
         footContactStates.put(robotSide, controllerToolbox.getFootContactState(robotSide));
      }

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
      bipedSupportPolygons.updateUsingContactStates(footContactStates);
      controllerToolbox.update();
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
