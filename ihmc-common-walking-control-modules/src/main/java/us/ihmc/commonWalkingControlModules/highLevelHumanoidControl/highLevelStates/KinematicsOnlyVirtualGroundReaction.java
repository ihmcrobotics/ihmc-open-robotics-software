package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KinematicsOnlyVirtualGroundReaction
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingHighLevelHumanoidController walkingController;
   private final SideDependentList<KinematicsOnlyContactStateHolder> contactStateHolders = new SideDependentList<>();
   private final FramePose3D desiredFootstep = new FramePose3D();
   private final InverseDynamicsCommandList inverseDynamicsContactHolderCommandList = new InverseDynamicsCommandList();

   public KinematicsOnlyVirtualGroundReaction(HighLevelHumanoidControllerToolbox controllerToolbox,
                                              StatusMessageOutputManager statusOutputManager,
                                              WalkingHighLevelHumanoidController walkingController)
   {
      this.controllerToolbox = controllerToolbox;
      this.walkingController = walkingController;

      statusOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, this::processFootstepStatus);

      for (RobotSide robotSide : RobotSide.values)
      {
         contactStateHolders.put(robotSide,
                                 KinematicsOnlyContactStateHolder.holdAtCurrent(controllerToolbox.getFootContactStates().get(robotSide)));
      }
   }

   private void processFootstepStatus(FootstepStatusMessage statusMessage)
   {
      RobotSide side = RobotSide.fromByte(statusMessage.getRobotSide());
      desiredFootstep.setIncludingFrame(worldFrame, statusMessage.getDesiredFootPositionInWorld(), statusMessage.getDesiredFootOrientationInWorld());

      switch (statusMessage.getFootstepStatus())
      {
         case FootstepStatusMessage.FOOTSTEP_STATUS_STARTED ->
         {
            contactStateHolders.remove(side);
            ((SettableFootSwitch) controllerToolbox.getFootSwitches().get(side)).setFootContactState(false);
         }
         case FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED ->
            contactStateHolders.put(side,
                                    new KinematicsOnlyContactStateHolder(controllerToolbox.getFootContactStates().get(side), desiredFootstep));

      }
   }

   public void update()
   {
      inverseDynamicsContactHolderCommandList.clear();
      for (RobotSide side : contactStateHolders.sides())
      {
         contactStateHolders.get(side).doControl();
         inverseDynamicsContactHolderCommandList.addCommand(contactStateHolders.get(side).getOutput());
      }

      // Trigger footstep completion based on swing time alone
      if (contactStateHolders.sides().length == 1 && walkingController.getBalanceManager().isICPPlanDone())
      {
         ((SettableFootSwitch) controllerToolbox.getFootSwitches().get(contactStateHolders.sides()[0].getOppositeSide())).setFootContactState(true);
      }
   }

   public InverseDynamicsCommandList getInverseDynamicsContactHolderCommandList()
   {
      return inverseDynamicsContactHolderCommandList;
   }
}
