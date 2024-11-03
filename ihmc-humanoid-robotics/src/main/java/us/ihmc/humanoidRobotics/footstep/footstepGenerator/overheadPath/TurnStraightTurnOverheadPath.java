package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.robotics.geometry.AngleTools;

public class TurnStraightTurnOverheadPath extends CompositeOverheadPath
{
   private TurningOverheadPath turningPath;
   private StraightLineOverheadPath straightLinePath;
   private TurningOverheadPath endTurningPath;
   private final static double defaultNoTranslationTolerance = 1e-14;

   public TurnStraightTurnOverheadPath(FramePose2D startPose, FramePose2D endPose, double headingOffset)
   {
      this(startPose, endPose, headingOffset, defaultNoTranslationTolerance);
   }

   public TurnStraightTurnOverheadPath(FramePose2D startPose, FramePose2D endPose, double headingOffset, double noTranslationTolerance)
   {
      super(calculatePaths(startPose, endPose, headingOffset, noTranslationTolerance));
      turningPath = (TurningOverheadPath) this.paths.get(0);
      straightLinePath = (StraightLineOverheadPath) this.paths.get(1);
      endTurningPath = (TurningOverheadPath) this.paths.get(2);
   }

   private static List<OverheadPath> calculatePaths(FramePose2D startPose, FramePose2D endPose, double headingOffset, double noTranslationTolerance)
   {
      startPose.checkReferenceFrameMatch(endPose);
      FramePoint2D endPosition = new FramePoint2D(endPose.getPosition());
      double heading = AngleTools.calculateHeading(startPose, endPosition, headingOffset, noTranslationTolerance);
      FrameOrientation2D intermediateOrientation = new FrameOrientation2D(startPose.getReferenceFrame(), heading);
      TurningOverheadPath turningPath = new TurningOverheadPath(startPose, intermediateOrientation);
      FramePose2D intermediatePose = turningPath.getPoseAtS(1.0);
      StraightLineOverheadPath straightPath = new StraightLineOverheadPath(intermediatePose, endPosition);
      intermediatePose = straightPath.getPoseAtS(1.0);
      FrameOrientation2D endOrientation = new FrameOrientation2D(endPose.getOrientation());
      TurningOverheadPath endTurningPath = new TurningOverheadPath(intermediatePose, endOrientation);

      List<OverheadPath> paths = new ArrayList<OverheadPath>();
      paths.add(turningPath);
      paths.add(straightPath);
      paths.add(endTurningPath);

      return paths;
   }

   public double getSignedTurningAmountInitialTurn()
   {
      return turningPath.getDeltaYaw();
   }

   public double getSignedTurningAmountFinalTurn()
   {
      return endTurningPath.getDeltaYaw();
   }

   public double getDistance()
   {
      return straightLinePath.getDistance();
   }
}
