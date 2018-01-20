package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.InterpolatedFootstepOverheadPath;
import us.ihmc.robotics.geometry.AngleTools;

/**
 * Created by agrabertilton on 3/6/15.
 */
public class TurnInPlaceFootstepOverheadPath extends FootstepOverheadPath
{
   FramePose2D startPose;
   FramePose2D endPose;
   double turnToStraightEquivalence;
   double distance;
   double angleChange;

   public TurnInPlaceFootstepOverheadPath(FramePose2D startPose, FramePose2D endPose){
      this(startPose, endPose, 1.0);
   }

   public TurnInPlaceFootstepOverheadPath(FramePose2D startPose, FramePose2D endPose, double turnToStraightEquivalence)
   {
      this.turnToStraightEquivalence = turnToStraightEquivalence;
      startPose.checkReferenceFrameMatch(endPose);
      this.startPose = new FramePose2D(startPose);
      this.endPose = new FramePose2D(endPose);
      double deltaX = endPose.getX() - startPose.getX();
      double deltaY = endPose.getY() - startPose.getY();
      double straightDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
      if (straightDistance > 1e-8){
         throw new RuntimeException(this.getClass().getSimpleName() + ": Not a pure turn");
      }
      angleChange = AngleTools.trimAngleMinusPiToPi(endPose.getYaw() - startPose.getYaw());
      distance = Math.abs(angleChange) * turnToStraightEquivalence;
   }

   @Override
   public FramePose2D getPoseAtDistance(double distanceAlongPath)
   {
      double interpolationValue = distanceAlongPath / distance;
      interpolationValue = Math.max(interpolationValue, 0.0);
      interpolationValue = Math.min(interpolationValue, 1.0);
      double x = (1-interpolationValue) * startPose.getX() + (interpolationValue) * endPose.getX();
      double y = (1-interpolationValue) * startPose.getY() + (interpolationValue) * endPose.getY();
      double yaw = AngleTools.trimAngleMinusPiToPi(startPose.getYaw() + (interpolationValue) * angleChange);
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
