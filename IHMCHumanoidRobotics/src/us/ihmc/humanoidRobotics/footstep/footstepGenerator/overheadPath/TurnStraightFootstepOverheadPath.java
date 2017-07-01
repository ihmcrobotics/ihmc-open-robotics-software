package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.CompositeFootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.InterpolatedFootstepOverheadPath;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose2d;

/**
 * Created by agrabertilton on 3/26/15.
 */
public class TurnStraightFootstepOverheadPath extends FootstepOverheadPath
{
   CompositeFootstepOverheadPath overheadPath;
   public TurnStraightFootstepOverheadPath(FramePose2d startPose, FramePose2d endPose)
   {
      this(startPose, endPose, 0.0);
   }

   public TurnStraightFootstepOverheadPath(FramePose2d startPose, FramePose2d endPose, double offsetAngle)
   {

      startPose.checkReferenceFrameMatch(endPose);

      double deltaX = endPose.getX() - startPose.getX();
      double deltaY = endPose.getY() - startPose.getY();
      double straightYaw = AngleTools.trimAngleMinusPiToPi(Math.atan2(deltaY, deltaX) + offsetAngle);

      FramePose2d straightStart = new FramePose2d(startPose);
      straightStart.setYaw(straightYaw);
      FramePose2d straightEnd = new FramePose2d(endPose);
      straightEnd.setYaw(straightYaw);

      overheadPath = new CompositeFootstepOverheadPath(new TurnInPlaceFootstepOverheadPath(startPose, straightStart));
      overheadPath.addPath(new InterpolatedFootstepOverheadPath(straightStart, straightEnd));
   }

   @Override
   public FramePose2d getPoseAtDistance(double distanceAlongPath)
   {
      return overheadPath.getPoseAtDistance(distanceAlongPath);
   }

   @Override
   public double getTotalDistance()
   {
      return overheadPath.getTotalDistance();
   }

   @Override
   public FootstepOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      return overheadPath.changeFrameCopy(desiredFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return overheadPath.getReferenceFrame();
   }
}
