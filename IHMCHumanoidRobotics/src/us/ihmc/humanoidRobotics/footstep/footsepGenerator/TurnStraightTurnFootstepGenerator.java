package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

import us.ihmc.humanoidRobotics.footstep.footsepGenerator.overheadPath.TurnStraightTurnOverheadPath;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TurnStraightTurnFootstepGenerator extends AbstractSimpleParametersFootstepGenerator
{
   private TurnStraightTurnOverheadPath footstepPath;
   private FramePose2d endPose;
   private double pathOrientation;

   public TurnStraightTurnFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FramePose2d endPose, PathTypeStepParameters pathType)
   {
      super(feet, soleFrames, pathType);
      this.endPose = endPose;
      this.pathOrientation = pathType.getAngle();
   }

   public TurnStraightTurnFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FramePose2d endPose, PathTypeStepParameters pathType,
           RobotSide stanceStart)
   {
      super(feet, soleFrames, pathType, stanceStart);
      this.endPose = endPose;
      this.pathOrientation = pathType.getAngle();
   }

   protected void initialize(FramePose2d startPose)
   {
      setFootstepPath(new TurnStraightTurnOverheadPath(startPose, endPose, pathOrientation, noTranslationTolerance));
      footstepCounter = new FootstepCounterForTurnStraightTurnPaths();
   }

   protected void initializeFootstepCounter(double openingAngle, double closingAngle, double stepLength, boolean hipExtensionFirst, boolean willSetFirstStraightStepToFarSideIfNoTurns, double standardFootWidth)
   {
      double totalAngleInitial = footstepPath.getSignedTurningAmountInitialTurn();
      double totalAngleFinal = footstepPath.getSignedTurningAmountFinalTurn();
      double totalDistance = footstepPath.getDistance();
      ((FootstepCounterForTurnStraightTurnPaths) footstepCounter).initialize(openingAngle, closingAngle, stepLength, totalAngleInitial, totalDistance,
              totalAngleFinal, hipExtensionFirst, isLeftRightPath, willSetFirstStraightStepToFarSideIfNoTurns, initialDeltaFeetX, initialDeltaFeetY, initialDeltaFeetYaw, standardFootWidth);
   }

   public double getSignedInitialTurnDirection()
   {
      return footstepPath.getSignedTurningAmountInitialTurn();
   }

   public void setFootstepPath(TurnStraightTurnOverheadPath footstepPath)
   {
      this.footstepPath = footstepPath;
   }

   protected TurnStraightTurnOverheadPath getPath()
   {
      return footstepPath;
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

      return (getDistance() > eps) || (Math.abs(getSignedInitialTurnDirection()) > eps);
   }
}
