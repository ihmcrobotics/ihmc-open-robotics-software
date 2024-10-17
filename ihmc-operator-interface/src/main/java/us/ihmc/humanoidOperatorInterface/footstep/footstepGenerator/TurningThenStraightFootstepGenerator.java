package us.ihmc.humanoidOperatorInterface.footstep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidOperatorInterface.footstep.footstepGenerator.overheadPath.TurnThenStraightOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.PathTypeStepParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class TurningThenStraightFootstepGenerator extends AbstractSimpleParametersFootstepGenerator
{
   private TurnThenStraightOverheadPath footstepPath;
   private FramePoint2D endPoint;
   private double pathOrientation;

   public TurningThenStraightFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, FramePoint2D endPoint, PathTypeStepParameters pathType)
   {
      super(feet, soleFrames, pathType);
      this.endPoint = endPoint;
      this.pathOrientation = pathType.getAngle();
   }

   public TurningThenStraightFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, FramePoint2D endPoint, PathTypeStepParameters pathType,
           RobotSide stanceStart)
   {
      super(feet, soleFrames, pathType, stanceStart);
      this.endPoint = endPoint;
      this.pathOrientation = pathType.getAngle();
   }

   protected void initialize(FramePose2D startPose)
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
