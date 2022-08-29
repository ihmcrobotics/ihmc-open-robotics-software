package us.ihmc.gdx.ui.affordances;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXFootInteractable extends GDXLiveRobotPartInteractable
{
   public static boolean collisionLinkIsFoot(RobotSide side, GDXRobotCollisionLink collisionLink, FullHumanoidRobotModel fullRobotModel)
   {
      return collisionLink.getRigidBodyName().equals(fullRobotModel.getFoot(side).getName());
   }

   public GDXFootInteractable(RobotSide side,
                              GDXImGuiBasedUI baseUI,
                              GDXRobotCollisionLink collisionLink,
                              DRCRobotModel robotModel,
                              FullHumanoidRobotModel fullRobotModel)
   {
      String modelFileName = GDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(collisionLink.getRigidBodyName()));
      create(collisionLink,
             fullRobotModel.getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
             modelFileName,
             baseUI.getPrimary3DPanel());
   }
}
