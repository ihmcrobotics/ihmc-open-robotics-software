package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class StraightLineOverheadPath extends OverheadPath
{
   private final FramePoint2d startPoint;
   private final FramePoint2d endPoint;
   private final FrameOrientation2d orientation;
   private final double distance;

   public StraightLineOverheadPath(FramePose2d startPose, FramePoint2d endPosition)
   {
      startPose.checkReferenceFrameMatch(endPosition);
      this.startPoint = new FramePoint2d();
      startPose.getPosition(startPoint);
      this.endPoint = new FramePoint2d(endPosition);
      this.orientation = new FrameOrientation2d();
      startPose.getOrientation(orientation);
      this.distance = endPoint.distance(startPoint);
   }

   private FramePoint2d tempPosition;

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
         tempPosition = new FramePoint2d(orientation.getReferenceFrame());
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
      FramePoint2d newStartPoint = new FramePoint2d(startPoint);
      newStartPoint.changeFrame(desiredFrame);
      FrameOrientation2d newStartOrientation = new FrameOrientation2d(orientation);
      newStartOrientation.changeFrame(desiredFrame);
      FramePose2d newStartPose = new FramePose2d(newStartPoint, newStartOrientation);
      FramePoint2d newEndPoint = new FramePoint2d(endPoint);
      newEndPoint.changeFrame(desiredFrame);

      return new StraightLineOverheadPath(newStartPose, newEndPoint);
   }

   public double getDistance()
   {
      return distance;
   }

}
