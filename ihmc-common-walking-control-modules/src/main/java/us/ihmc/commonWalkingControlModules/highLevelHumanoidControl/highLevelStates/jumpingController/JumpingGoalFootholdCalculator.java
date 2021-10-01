package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingControllerToolbox.worldFrame;

public class JumpingGoalFootholdCalculator
{
   private final FramePose3D midFootPose = new FramePose3D();
   private final FramePose3D goalPose = new FramePose3D();

   private final PoseReferenceFrame goalPoseFrame = new PoseReferenceFrame("goalPoseFrame", ReferenceFrame.getWorldFrame());
   private final SideDependentList<FramePose3D> footGoalPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   public void computeGoalPose(ReferenceFrame midstanceZUpFrame, double goalLength, double goalWidth, double goalHeight, double goalRotation)
   {
      midFootPose.setToZero(midstanceZUpFrame);
      goalPose.setIncludingFrame(midFootPose);
      goalPose.setX(goalLength);
      if (!Double.isNaN(goalHeight))
         goalPose.setZ(goalHeight);
      if (!Double.isNaN(goalRotation))
         goalPose.getOrientation().setToYawOrientation(goalRotation);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      goalPoseFrame.setPoseAndUpdate(goalPose);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footGoalPose = footGoalPoses.get(robotSide);
         footGoalPose.setToZero(goalPoseFrame);
         double width = 0.5 * goalWidth;
         width = robotSide.negateIfRightSide(width);

         footGoalPose.setY(width);
         footGoalPose.changeFrame(worldFrame);
      }
   }

   public FramePose3DReadOnly getGoalPose()
   {
      return goalPose;
   }

   public FramePose3DReadOnly getFootGoalPose(RobotSide robotSide)
   {
      return footGoalPoses.get(robotSide);
   }
}
