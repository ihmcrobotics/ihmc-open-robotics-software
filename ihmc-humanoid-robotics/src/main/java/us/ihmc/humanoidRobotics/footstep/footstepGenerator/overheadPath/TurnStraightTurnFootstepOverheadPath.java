package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.CompositeFootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.InterpolatedFootstepOverheadPath;
import us.ihmc.robotics.geometry.AngleTools;

/**
 * Created by agrabertilton on 3/6/15.
 */
public class TurnStraightTurnFootstepOverheadPath extends FootstepOverheadPath
{
   CompositeFootstepOverheadPath overheadPath;
   public TurnStraightTurnFootstepOverheadPath(FramePose2D startPose, FramePose2D endPose)
   {
      this(startPose, endPose, 0.0);
   }

   public TurnStraightTurnFootstepOverheadPath(FramePose2D startPose, FramePose2D endPose, double offsetAngle)
   {

      startPose.checkReferenceFrameMatch(endPose);

      double deltaX = endPose.getX() - startPose.getX();
      double deltaY = endPose.getY() - startPose.getY();
      double straightYaw = AngleTools.trimAngleMinusPiToPi(Math.atan2(deltaY, deltaX) + offsetAngle);

      FramePose2D straightStart = new FramePose2D(startPose);
      straightStart.setYaw(straightYaw);
      FramePose2D straightEnd = new FramePose2D(endPose);
      straightEnd.setYaw(straightYaw);

      overheadPath = new CompositeFootstepOverheadPath(new TurnInPlaceFootstepOverheadPath(startPose, straightStart));
      overheadPath.addPath(new InterpolatedFootstepOverheadPath(straightStart, straightEnd));
      overheadPath.addPath(new TurnInPlaceFootstepOverheadPath(straightEnd, endPose));
   }

   @Override
   public FramePose2D getPoseAtDistance(double distanceAlongPath)
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
