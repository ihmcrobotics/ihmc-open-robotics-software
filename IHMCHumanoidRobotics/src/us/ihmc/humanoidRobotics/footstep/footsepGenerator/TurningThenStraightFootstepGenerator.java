package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

import us.ihmc.humanoidRobotics.footstep.footsepGenerator.overheadPath.TurnThenStraightOverheadPath;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TurningThenStraightFootstepGenerator extends AbstractSimpleParametersFootstepGenerator
{
   private TurnThenStraightOverheadPath footstepPath;
   private FramePoint2d endPoint;
   private double pathOrientation;

   public TurningThenStraightFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FramePoint2d endPoint, PathTypeStepParameters pathType)
   {
      super(feet, soleFrames, pathType);
      this.endPoint = endPoint;
      this.pathOrientation = pathType.getAngle();
   }

   public TurningThenStraightFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FramePoint2d endPoint, PathTypeStepParameters pathType,
           RobotSide stanceStart)
   {
      super(feet, soleFrames, pathType, stanceStart);
      this.endPoint = endPoint;
      this.pathOrientation = pathType.getAngle();
   }

   protected void initialize(FramePose2d startPose)
   {
      setFootstepPath(new TurnThenStraightOverheadPath(startPose, endPoint, pathOrientation, noTranslationTolerance));
      footstepCounter = new FootstepCounterForTurnThenStraightPaths();
   }

   protected void initializeFootstepCounter(double openingAngle, double closingAngle, double stepLength, boolean hipExtensionFirst, boolean willSetFirstStraightStepToFarSideIfNoTurns, double standardFootWidth)
   {
      double totalAngle = footstepPath.getSignedTurningAmount();
      double totalDistance = footstepPath.getDistance();
      ((FootstepCounterForTurnThenStraightPaths) footstepCounter).initialize(openingAngle, closingAngle, stepLength, totalAngle, totalDistance,
              hipExtensionFirst, isLeftRightPath, willSetFirstStraightStepToFarSideIfNoTurns, initialDeltaFeetX, initialDeltaFeetY, initialDeltaFeetYaw, standardFootWidth);
   }

   protected double getSignedInitialTurnDirection()
   {
      return footstepPath.getSignedTurningAmount();
   }

   public void setFootstepPath(TurnThenStraightOverheadPath footstepPath)
   {
      this.footstepPath = footstepPath;
   }

   protected TurnThenStraightOverheadPath getPath()
   {
      return footstepPath;
   }

   public double getTurningAmount()
   {
      if (footstepPath == null)
         initialize();

      return footstepPath.getSignedTurningAmount();
   }

   public double getDistance()
   {
      if (footstepPath == null)
         initialize();

      return footstepPath.getDistance();
   }
   
   public boolean hasDisplacement()
   {
      double eps = noTranslationTolerance;

      return (getDistance() > eps) || (Math.abs(getTurningAmount()) > eps);
   }
}
