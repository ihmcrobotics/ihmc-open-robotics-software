package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePose2d;

public class StraightLineOverheadPath extends OverheadPath
{
   private final FramePoint2D startPoint;
   private final FramePoint2D endPoint;
   private final FrameOrientation2d orientation;
   private final double distance;

   public StraightLineOverheadPath(FramePose2d startPose, FramePoint2D endPosition)
   {
      startPose.checkReferenceFrameMatch(endPosition);
      this.startPoint = new FramePoint2D();
      startPose.getPositionIncludingFrame(startPoint);
      this.endPoint = new FramePoint2D(endPosition);
      this.orientation = new FrameOrientation2d();
      startPose.getOrientationIncludingFrame(orientation);
      this.distance = endPoint.distance(startPoint);
   }

   private FramePoint2D tempPosition;

   @Override
   public FramePose2d getPoseAtS(double pathVariableS)
   {
      pathVariableS = MathTools.clamp(pathVariableS, 0.0, 1.0);
      return getExtrapolatedPoseAtS(pathVariableS);
   }

   @Override
   public FramePose2d getExtrapolatedPoseAtS(double pathVariableS)
   {
      if (tempPosition == null)
         tempPosition = new FramePoint2D(orientation.getReferenceFrame());
      tempPosition.interpolate(startPoint, endPoint, pathVariableS);

      return new FramePose2d(tempPosition, orientation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return orientation.getReferenceFrame();
   }

   @Override
   public StraightLineOverheadPath changeFrameCopy(ReferenceFrame desiredFrame)
   {
      FramePoint2D newStartPoint = new FramePoint2D(startPoint);
      newStartPoint.changeFrame(desiredFrame);
      FrameOrientation2d newStartOrientation = new FrameOrientation2d(orientation);
      newStartOrientation.changeFrame(desiredFrame);
      FramePose2d newStartPose = new FramePose2d(newStartPoint, newStartOrientation);
      FramePoint2D newEndPoint = new FramePoint2D(endPoint);
      newEndPoint.changeFrame(desiredFrame);

      return new StraightLineOverheadPath(newStartPose, newEndPoint);
   }

   public double getDistance()
   {
      return distance;
   }

}
