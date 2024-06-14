package us.ihmc.behaviors.roomExploration.old;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

class BuildingExplorationBehaviorTeleopState implements State
{
   private final BehaviorHelper helper;

   public BuildingExplorationBehaviorTeleopState(BehaviorHelper helper)
   {
      this.helper = helper;
   }

   @Override
   public void onEntry()
   {
      // do nothing, this is an idle state where the operator sends commands from the VR UI
      LogTools.info("Entering " + getClass().getSimpleName());
   }

   @Override
   public void doAction(double timeInState)
   {
      // do nothing
   }

   @Override
   public void onExit(double timeInState)
   {
      LogTools.info("Exiting " + getClass().getSimpleName());

      double trajectoryTime = 3.5;
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      homeLeftArm.setTrajectoryTime(trajectoryTime);
      helper.publishToController(homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
      homeRightArm.setTrajectoryTime(trajectoryTime);
      helper.publishToController(homeRightArm);

      GoHomeMessage homePelvis = new GoHomeMessage();
      homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
      homePelvis.setTrajectoryTime(trajectoryTime);
      helper.publishToController(homePelvis);

      GoHomeMessage homeChest = new GoHomeMessage();
      homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
      homeChest.setTrajectoryTime(trajectoryTime);
      helper.publishToController(homeChest);
   }
}
