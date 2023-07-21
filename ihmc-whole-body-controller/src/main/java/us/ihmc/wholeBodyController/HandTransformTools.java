package us.ihmc.wholeBodyController;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandTransformTools
{
   public static void getHandGraphicToControlFrameTransform(FullHumanoidRobotModel fullRobotModel,
                                                            UIParameters uiParameters,
                                                            RobotSide side,
                                                            RigidBodyTransform graphicToControlFrameTransform)
   {
      RigidBodyTransform controlToHandFrameTransform = fullRobotModel.getHandControlFrame(side).getTransformToParent();
      RigidBodyTransform graphicToHandFrameTransform = uiParameters.getHandGraphicToHandFrameTransform(side);

      ReferenceFrame handFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame controlFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(handFrame, controlToHandFrameTransform);
      ReferenceFrame graphicFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(handFrame, graphicToHandFrameTransform);

      FramePose3D graphicPose = new FramePose3D(graphicFrame);
      graphicPose.changeFrame(controlFrame);

      graphicPose.get(graphicToControlFrameTransform);

      controlFrame.remove();
      graphicFrame.remove();
   }

   public static void getHandControlToBodyFixedCoMFrameTransform(FullHumanoidRobotModel fullRobotModel,
                                                                 RobotSide side,
                                                                 RigidBodyTransform controlToBodyFixedCoMFrameTransform)
   {
      RigidBodyTransform controlToHandFrameTransform = fullRobotModel.getHandControlFrame(side).getTransformToParent();
      RigidBodyTransform bodyFixedCoMToHandFrameTransform = fullRobotModel.getHand(side).getBodyFixedFrame().getTransformToParent();

      ReferenceFrame handFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame controlFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(handFrame, controlToHandFrameTransform);
      ReferenceFrame bodyFixedCoMFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(handFrame, bodyFixedCoMToHandFrameTransform);

      FramePose3D controlPose = new FramePose3D(controlFrame);
      controlPose.changeFrame(bodyFixedCoMFrame);

      controlPose.get(controlToBodyFixedCoMFrameTransform);

      controlFrame.remove();
      bodyFixedCoMFrame.remove();
   }

   public static void getHandLinkToControlFrameTransform(FullHumanoidRobotModel fullRobotModel, RobotSide side, RigidBodyTransform linkToControlFrameTransform)
   {
      linkToControlFrameTransform.set(fullRobotModel.getHandControlFrame(side).getTransformToParent());
      linkToControlFrameTransform.invert();
   }
}
