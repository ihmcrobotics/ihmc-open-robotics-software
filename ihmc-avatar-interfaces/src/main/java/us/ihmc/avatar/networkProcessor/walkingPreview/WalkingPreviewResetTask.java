package us.ihmc.avatar.networkProcessor.walkingPreview;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WalkingPreviewResetTask implements WalkingPreviewTask
{
   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<WalkingPreviewContactStateHolder> contactStateHolders = new SideDependentList<>();
   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   private final WalkingHighLevelHumanoidController walkingController;

   private int count = 0;
   private final int numberOfTicksBeforeDone = 2; // 2 ticks seem necessary when reinitializing the controller.

   public WalkingPreviewResetTask(WalkingHighLevelHumanoidController walkingController, SideDependentList<YoPlaneContactState> footContactStates)
   {
      this.walkingController = walkingController;
      this.footContactStates = footContactStates;
   }

   @Override
   public void onEntry()
   {
      walkingController.requestImmediateTransitionToStandingAndHoldCurrent();

      for (RobotSide robotSide : RobotSide.values)
         contactStateHolders.put(robotSide, WalkingPreviewContactStateHolder.holdAtCurrent(footContactStates.get(robotSide)));
   }

   @Override
   public void doAction(double timeInState)
   {
      commandList.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         contactStateHolders.get(robotSide).doControl();
         commandList.addCommand(contactStateHolders.get(robotSide).getOutput());
      }

      count++;
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return count >= numberOfTicksBeforeDone;
   }

   @Override
   public InverseDynamicsCommand<?> getOutput()
   {
      return commandList;
   }
}
