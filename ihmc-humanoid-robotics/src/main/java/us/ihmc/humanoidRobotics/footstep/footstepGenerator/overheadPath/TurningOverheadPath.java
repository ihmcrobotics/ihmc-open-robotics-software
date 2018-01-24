package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;

public class TurningOverheadPath extends OverheadPath
{
   private final FrameOrientation2D startOrientation;
   private final FrameOrientation2D endOrientation;
   private final double deltaYaw;
   private final FramePoint2D point;

   public TurningOverheadPath(FramePose2D startPose, FrameOrientation2D endOrientation)
   {
      startPose.checkReferenceFrameMatch(endOrientation);
      this.startOrientation = new FrameOrientation2D(startPose.getOrientation());
      this.endOrientation = new FrameOrientation2D(endOrientation);
      this.point = new FramePoint2D(startPose.getPosition());
      this.deltaYaw = endOrientation.difference(startOrientation);
   }

   private FrameOrientation2D tempOrientation;

   public double getDeltaYaw()
   {
      return deltaYaw;//signed
   }

   @Override
   public FramePose2D getPoseAtS(double pathVariableS)
   {
      pathVariableS = MathTools.clamp(pathVariableS, 0.0, 1.0);
      return getExtrapolatedPoseAtS(pathVariableS);
   }

   @Override
   public FramePose2D getExtrapolatedPoseAtS(double pathVariableS)
   {
      if (tempOrientation == null)
         tempOrientation = new FrameOrientation2D(point.getReferenceFrame());
      tempOrientation.interpolate(startOrientation, endOrientation, pathVariableS);
      return new FramePose2D(point, tempOrientation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return point.getReferenceFrame();
   }

   @Override
   public TurningOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      FramePoint2D newStartPoint = new FramePoint2D(point);
      newStartPoint.changeFrame(desiredFrame);
      FrameOrientation2D newStartOrientation = new FrameOrientation2D(startOrientation);
      newStartOrientation.changeFrame(desiredFrame);
      FramePose2D newStartPose = new FramePose2D(newStartPoint, newStartOrientation);
      FrameOrientation2D newEndOrientation = new FrameOrientation2D(endOrientation);
      newEndOrientation.changeFrame(desiredFrame);
      return new TurningOverheadPath(newStartPose, newEndOrientation);
   }

}
