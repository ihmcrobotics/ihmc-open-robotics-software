package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.robotics.geometry.AngleTools;

public class TurnThenStraightOverheadPath extends CompositeOverheadPath
{
   private TurningOverheadPath turningPath;
   private StraightLineOverheadPath straightLinePath;
   private final static double defaultNoTranslationTolerance = 1e-14;

   public TurnThenStraightOverheadPath(FramePose2D startPose, FramePoint2D endPoint, double headingOffsetFromPath)
   {
      this(startPose, endPoint, headingOffsetFromPath, defaultNoTranslationTolerance);
   }

   public TurnThenStraightOverheadPath(FramePose2D startPose, FramePoint2D endPoint, double headingOffsetFromPath, double noTranslationTolerance)
   {
      super(calculatePaths(startPose, endPoint, headingOffsetFromPath, noTranslationTolerance));
      turningPath = (TurningOverheadPath) this.paths.get(0);
      straightLinePath = (StraightLineOverheadPath) this.paths.get(1);
   }

   private static List<OverheadPath> calculatePaths(FramePose2D startPose, FramePoint2D endPoint, double headingOffset, double noTranslationTolerance)
   {
      startPose.checkReferenceFrameMatch(endPoint);
      double heading = AngleTools.calculateHeading(startPose, endPoint, headingOffset, noTranslationTolerance);
      FrameOrientation2D intermediateOrientation = new FrameOrientation2D(startPose.getReferenceFrame(), heading);
      TurningOverheadPath turningPath = new TurningOverheadPath(startPose, intermediateOrientation);
      FramePose2D intermediatePose = turningPath.getPoseAtS(1.0);
      StraightLineOverheadPath straightPath = new StraightLineOverheadPath(intermediatePose, endPoint);
      List<OverheadPath> paths = new ArrayList<OverheadPath>();
      paths.add(turningPath);
      paths.add(straightPath);

      return paths;
   }

   public double getSignedTurningAmount()
   {
      return turningPath.getDeltaYaw();//signed
   }

   public double getDistance()
   {
      return straightLinePath.getDistance();
   }
}
