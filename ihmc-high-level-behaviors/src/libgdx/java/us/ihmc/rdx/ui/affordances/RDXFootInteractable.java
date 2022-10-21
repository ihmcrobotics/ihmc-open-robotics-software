package us.ihmc.rdx.ui.affordances;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXFootInteractable extends RDXLiveRobotPartInteractable
{
   public static boolean collisionLinkIsFoot(RobotSide side, RDXRobotCollisionLink collisionLink, FullHumanoidRobotModel fullRobotModel)
   {
      return collisionLink.getRigidBodyName().equals(fullRobotModel.getFoot(side).getName());
   }

   public RDXFootInteractable(RobotSide side,
                              RDXBaseUI baseUI,
                              RDXRobotCollisionLink collisionLink,
                              DRCRobotModel robotModel,
                              FullHumanoidRobotModel fullRobotModel)
   {
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(collisionLink.getRigidBodyName()));
      create(collisionLink,
             fullRobotModel.getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
             modelFileName,
             baseUI.getPrimary3DPanel());
   }
}
