package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created by agrabertilton on 2/20/15.
 */
public class InterpolatedFootstepOverheadPath extends FootstepOverheadPath
{
   FramePose2d startPose;
   FramePose2d endPose;
   double distance;

   public InterpolatedFootstepOverheadPath(FramePose2d startPose, FramePose2d endPose)
   {
      startPose.checkReferenceFrameMatch(endPose);
      this.startPose = new FramePose2d(startPose);
      this.endPose = new FramePose2d(endPose);
      double deltaX = endPose.getX() - startPose.getX();
      double deltaY = endPose.getY() - startPose.getY();
      distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
   }

   @Override
   public FramePose2d getPoseAtDistance(double distanceAlongPath)
   {
      double interpolationValue = distanceAlongPath / distance;
      interpolationValue = Math.max(interpolationValue, 0.0);
      interpolationValue = Math.min(interpolationValue, 1.0);
      double x = (1-interpolationValue) * startPose.getX() + (interpolationValue) * endPose.getX();
      double y = (1-interpolationValue) * startPose.getY() + (interpolationValue) * endPose.getY();
      double yaw = (1-interpolationValue) * startPose.getYaw() + (interpolationValue) * endPose.getYaw();
      return new FramePose2d(startPose.getReferenceFrame(), new Point2D(x,y), yaw);
   }

   @Override
   public double getTotalDistance()
   {
      return distance;
   }

   @Override
   public FootstepOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      FramePose2d newStartPose = new FramePose2d(startPose);
      newStartPose.changeFrame(desiredFrame);
      FramePose2d newEndPose = new FramePose2d(endPose);
      newEndPose.changeFrame(desiredFrame);
      return new InterpolatedFootstepOverheadPath(newStartPose, newEndPose);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return startPose.getReferenceFrame();
   }
}
