package us.ihmc.wholeBodyController.diagnostics;

import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.wholeBodyController.DRCOutputWriterWithTorqueOffsets;

public class DiagnosticsWhenHangingControllerFactory implements HighLevelBehaviorFactory
{
   private FullRobotModel fullRobotModel;
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private DRCOutputWriterWithTorqueOffsets outputWriterWithTorqueOffsets;
   private boolean transitionRequested = false;

   private final boolean useArms;
   private final boolean robotIsHanging;
   private final TorqueOffsetPrinter torqueOffsetPrinter;
   
   private final HumanoidJointPoseList humanoidJointPoseList;

   public DiagnosticsWhenHangingControllerFactory(HumanoidJointPoseList humanoidJointPoseList, boolean useArms, boolean robotIsHanging, TorqueOffsetPrinter torqueOffsetPrinter)
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

   public void attachOutputWriterWithTorqueOffsets(DRCOutputWriterWithTorqueOffsets outputWriterWithTorqueOffsets)
   {
      if (controller != null)
         controller.attachOutputWriterWithTorqueOffsets(outputWriterWithTorqueOffsets);
      else
         this.outputWriterWithTorqueOffsets = outputWriterWithTorqueOffsets;
   }

   private DiagnosticsWhenHangingController controller;

   public DiagnosticsWhenHangingController getController()
   {
      return controller;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      controller = new DiagnosticsWhenHangingController(humanoidJointPoseList, useArms, robotIsHanging, momentumBasedController, this.torqueOffsetPrinter);

      controller.addUpdatables(updatables);

      if (outputWriterWithTorqueOffsets != null)
         controller.attachOutputWriterWithTorqueOffsets(outputWriterWithTorqueOffsets);

      return controller;
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }

   public void setTransitionRequested(boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
   }
}
