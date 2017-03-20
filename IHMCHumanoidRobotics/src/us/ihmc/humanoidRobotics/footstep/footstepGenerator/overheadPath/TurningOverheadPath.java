package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TurningOverheadPath extends OverheadPath
{
   private final FrameOrientation2d startOrientation;
   private final FrameOrientation2d endOrientation;
   private final double deltaYaw;
   private final FramePoint2d point;

   public TurningOverheadPath(FramePose2d startPose, FrameOrientation2d endOrientation)
   {
      startPose.checkReferenceFrameMatch(endOrientation);
      this.startOrientation = new FrameOrientation2d();
      startPose.getOrientation(startOrientation);
      this.endOrientation = new FrameOrientation2d(endOrientation);
      this.point = new FramePoint2d();
      startPose.getPosition(point);
      this.deltaYaw = endOrientation.sub(startOrientation);
   }

   private FrameOrientation2d tempOrientation;

   public double getDeltaYaw()
   {
      return deltaYaw;//signed
   }

   @Override
   public FramePose2d getPoseAtS(double pathVariableS)
   {
      pathVariableS = MathTools.clamp(pathVariableS, 0.0, 1.0);
      return getExtrapolatedPoseAtS(pathVariableS);
   }

   @Override
   public FramePose2d getExtrapolatedPoseAtS(double pathVariableS)
   {
      if (tempOrientation == null)
         tempOrientation = new FrameOrientation2d(point.getReferenceFrame());
      tempOrientation.extrapolate(startOrientation, endOrientation, pathVariableS);
      return new FramePose2d(point, tempOrientation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return point.getReferenceFrame();
   }

   @Override
   public TurningOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      FramePoint2d newStartPoint = new FramePoint2d(point);
      newStartPoint.changeFrame(desiredFrame);
      FrameOrientation2d newStartOrientation = new FrameOrientation2d(startOrientation);
      newStartOrientation.changeFrame(desiredFrame);
      FramePose2d newStartPose = new FramePose2d(newStartPoint, newStartOrientation);
      FrameOrientation2d newEndOrientation = new FrameOrientation2d(endOrientation);
      newEndOrientation.changeFrame(desiredFrame);
      return new TurningOverheadPath(newStartPose, newEndOrientation);
   }

}
