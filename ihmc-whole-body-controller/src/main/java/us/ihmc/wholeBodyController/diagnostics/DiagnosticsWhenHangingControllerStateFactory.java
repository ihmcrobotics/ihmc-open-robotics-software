package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;

public class DiagnosticsWhenHangingControllerStateFactory implements HighLevelControllerStateFactory
{
   private FullRobotModel fullRobotModel;
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private JointTorqueOffsetProcessor jointTorqueOffsetProcessor;
   private boolean transitionRequested = false;

   private final boolean useArms;
   private final boolean robotIsHanging;
   private final TorqueOffsetPrinter torqueOffsetPrinter;
   
   private final HumanoidJointPoseList humanoidJointPoseList;
   private DiagnosticsWhenHangingControllerState controller;

   public DiagnosticsWhenHangingControllerStateFactory(HumanoidJointPoseList humanoidJointPoseList, boolean useArms, boolean robotIsHanging, TorqueOffsetPrinter torqueOffsetPrinter)
   {
      this.humanoidJointPoseList = humanoidJointPoseList;
      this.useArms = useArms;
      this.robotIsHanging = robotIsHanging;
      this.torqueOffsetPrinter = torqueOffsetPrinter;
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public void attachJointTorqueOffsetProcessor(JointTorqueOffsetProcessor jointTorqueOffsetProcessor)
   {
      if (controller != null)
         controller.attachjointTorqueOffsetProcessor(jointTorqueOffsetProcessor);
      else
         this.jointTorqueOffsetProcessor = jointTorqueOffsetProcessor;
   }


   public DiagnosticsWhenHangingControllerState getController()
   {
      return controller;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      controller = new DiagnosticsWhenHangingControllerState(humanoidJointPoseList, useArms, robotIsHanging,
                                                             controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                             controllerFactoryHelper.getHighLevelControllerParameters(), this.torqueOffsetPrinter);

      controller.addUpdatables(updatables);

      if (jointTorqueOffsetProcessor != null)
         controller.attachjointTorqueOffsetProcessor(jointTorqueOffsetProcessor);

      return controller;
   }

   @Override
   public boolean isTransitionToControllerRequested()
   {
      return transitionRequested;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.DIAGNOSTICS;
   }

   public void setTransitionRequested(boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
   }
}
