package us.ihmc.humanoidRobotics.footstep.footsepGenerator.overheadPath;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;

public class TurnStraightTurnOverheadPath extends CompositeOverheadPath
{
   private TurningOverheadPath turningPath;
   private StraightLineOverheadPath straightLinePath;
   private TurningOverheadPath endTurningPath;
   private final static double defaultNoTranslationTolerance = 1e-14;

   public TurnStraightTurnOverheadPath(FramePose2d startPose, FramePose2d endPose, double headingOffset)
   {
      this(startPose, endPose, headingOffset, defaultNoTranslationTolerance);
   }

   public TurnStraightTurnOverheadPath(FramePose2d startPose, FramePose2d endPose, double headingOffset, double noTranslationTolerance)
   {
      super(calculatePaths(startPose, endPose, headingOffset, noTranslationTolerance));
      turningPath = (TurningOverheadPath) this.paths.get(0);
      straightLinePath = (StraightLineOverheadPath) this.paths.get(1);
      endTurningPath = (TurningOverheadPath) this.paths.get(2);
   }

   private static List<OverheadPath> calculatePaths(FramePose2d startPose, FramePose2d endPose, double headingOffset, double noTranslationTolerance)
   {
      startPose.checkReferenceFrameMatch(endPose);
      FramePoint2d endPosition = new FramePoint2d();
      endPose.getPosition(endPosition);
      double heading = AngleTools.calculateHeading(startPose, endPosition, headingOffset, noTranslationTolerance);
      FrameOrientation2d intermediateOrientation = new FrameOrientation2d(startPose.getReferenceFrame(), heading);
      TurningOverheadPath turningPath = new TurningOverheadPath(startPose, intermediateOrientation);
      FramePose2d intermediatePose = turningPath.getPoseAtS(1.0);
      StraightLineOverheadPath straightPath = new StraightLineOverheadPath(intermediatePose, endPosition);
      intermediatePose = straightPath.getPoseAtS(1.0);
      FrameOrientation2d endOrientation = new FrameOrientation2d();
      endPose.getOrientation(endOrientation);
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
