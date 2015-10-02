package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.humanoidRobotics.footstep.footstepGenerator.CompositeFootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.InterpolatedFootstepOverheadPath;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created by agrabertilton on 3/6/15.
 */
public class TurnStraightTurnFootstepOverheadPath extends FootstepOverheadPath
{
   CompositeFootstepOverheadPath overheadPath;
   public TurnStraightTurnFootstepOverheadPath(FramePose2d startPose, FramePose2d endPose)
   {
      this(startPose, endPose, 0.0);
   }

   public TurnStraightTurnFootstepOverheadPath(FramePose2d startPose, FramePose2d endPose, double offsetAngle)
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
      overheadPath.addPath(new TurnInPlaceFootstepOverheadPath(straightEnd, endPose));
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
