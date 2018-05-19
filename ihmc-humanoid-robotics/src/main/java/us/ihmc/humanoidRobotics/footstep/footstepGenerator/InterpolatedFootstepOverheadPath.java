package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;

/**
 * Created by agrabertilton on 2/20/15.
 */
public class InterpolatedFootstepOverheadPath extends FootstepOverheadPath
{
   FramePose2D startPose;
   FramePose2D endPose;
   double distance;

   public InterpolatedFootstepOverheadPath(FramePose2D startPose, FramePose2D endPose)
   {
      startPose.checkReferenceFrameMatch(endPose);
      this.startPose = new FramePose2D(startPose);
      this.endPose = new FramePose2D(endPose);
      double deltaX = endPose.getX() - startPose.getX();
      double deltaY = endPose.getY() - startPose.getY();
      distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
   }

   @Override
   public FramePose2D getPoseAtDistance(double distanceAlongPath)
   {
      double interpolationValue = distanceAlongPath / distance;
      interpolationValue = Math.max(interpolationValue, 0.0);
      interpolationValue = Math.min(interpolationValue, 1.0);
      double x = (1-interpolationValue) * startPose.getX() + (interpolationValue) * endPose.getX();
      double y = (1-interpolationValue) * startPose.getY() + (interpolationValue) * endPose.getY();
      double yaw = (1-interpolationValue) * startPose.getYaw() + (interpolationValue) * endPose.getYaw();
      return new FramePose2D(startPose.getReferenceFrame(), new Point2D(x,y), yaw);
   }

   @Override
   public double getTotalDistance()
   {
      return distance;
   }

   @Override
   public FootstepOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      FramePose2D newStartPose = new FramePose2D(startPose);
      newStartPose.changeFrame(desiredFrame);
      FramePose2D newEndPose = new FramePose2D(endPose);
      newEndPose.changeFrame(desiredFrame);
      return new InterpolatedFootstepOverheadPath(newStartPose, newEndPose);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return startPose.getReferenceFrame();
   }
}
