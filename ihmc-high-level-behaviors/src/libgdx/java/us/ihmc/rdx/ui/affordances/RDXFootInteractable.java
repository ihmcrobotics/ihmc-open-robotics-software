package us.ihmc.rdx.ui.affordances;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXFootInteractable extends RDXLiveRobotPartInteractable
{
   public static boolean robotCollidableIsFoot(RobotSide side, RDXRobotCollidable robotCollidable, FullHumanoidRobotModel fullRobotModel)
   {
      return robotCollidable.getRigidBodyName().equals(fullRobotModel.getFoot(side).getName());
   }

   public RDXFootInteractable(RobotSide side,
                              RDXBaseUI baseUI,
                              RDXRobotCollidable robotCollidable,
                              DRCRobotModel robotModel,
                              FullHumanoidRobotModel fullRobotModel)
   {
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(robotCollidable.getRigidBodyName()));
      create(robotCollidable,
             fullRobotModel.getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
             modelFileName,
             baseUI.getPrimary3DPanel());
   }
}
