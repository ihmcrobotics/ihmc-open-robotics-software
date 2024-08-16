package us.ihmc.rdx.ui.affordances;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * UI for selecting and moving a humanoid robot's foot.
 * See {@link RDXInteractableRobotLink} for more explanation.
 */
public class RDXInteractableFoot extends RDXInteractableRobotLink
{
   public static boolean robotCollidableIsFoot(RobotSide side, RDXRobotCollidable robotCollidable, FullHumanoidRobotModel fullRobotModel)
   {
      return robotCollidable.getRigidBodyName().equals(fullRobotModel.getFoot(side).getName());
   }

   public RDXInteractableFoot(RobotSide side,
                              RDXBaseUI baseUI,
                              RDXRobotCollidable robotCollidable,
                              DRCRobotModel robotModel,
                              FullHumanoidRobotModel fullRobotModel)
   {
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(robotCollidable.getRigidBodyName()));

      RigidBodyTransform graphicToControlFrameTransform = new RigidBodyTransform();
      RigidBodyTransform linkToControlFrameTransform = new RigidBodyTransform();

      FramePose3D graphicFrame = new FramePose3D();
      graphicFrame.setToZero(fullRobotModel.getFoot(side).getParentJoint().getFrameAfterJoint());
      graphicFrame.changeFrame(fullRobotModel.getSoleFrame(side));
      graphicFrame.get(graphicToControlFrameTransform);
      graphicFrame.get(linkToControlFrameTransform);

      create(robotCollidable,
             fullRobotModel.getSoleFrame(side),
             graphicToControlFrameTransform,
             linkToControlFrameTransform,
             modelFileName,
             baseUI.getPrimary3DPanel());
   }
}
