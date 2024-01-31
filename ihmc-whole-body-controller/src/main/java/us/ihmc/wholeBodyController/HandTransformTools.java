package us.ihmc.wholeBodyController;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTransformTools
{
   public static RigidBodyTransformReadOnly getHandGraphicToControlFrameTransform(FullHumanoidRobotModel fullRobotModel,
                                                                                  UIParameters uiParameters,
                                                                                  RobotSide side)
   {
      RigidBodyTransform graphicToHandFrameTransform = uiParameters.getHandGraphicToHandFrameTransform(side);

      ReferenceFrame handFrame = fullRobotModel.getHand(side).getParentJoint().getFrameAfterJoint();
      ReferenceFrame controlFrame = fullRobotModel.getHandControlFrame(side);

      FramePose3D graphicPose = new FramePose3D(handFrame);
      graphicPose.applyInverseTransform(graphicToHandFrameTransform);
      graphicPose.changeFrame(controlFrame);

      return graphicPose;
   }

   public static RigidBodyTransformReadOnly getHandLinkToControlFrameTransform(FullHumanoidRobotModel fullRobotModel, RobotSide side)
   {
      RigidBodyTransform linkToControlFrameTransform = new RigidBodyTransform();
      getHandLinkToControlFrameTransform(fullRobotModel, side, linkToControlFrameTransform);
      return linkToControlFrameTransform;
   }

   public static void getHandLinkToControlFrameTransform(FullHumanoidRobotModel fullRobotModel, RobotSide side, RigidBodyTransform linkToControlFrameTransform)
   {
      linkToControlFrameTransform.set(fullRobotModel.getHandControlFrame(side).getTransformToParent());
      linkToControlFrameTransform.invert();
   }
}
