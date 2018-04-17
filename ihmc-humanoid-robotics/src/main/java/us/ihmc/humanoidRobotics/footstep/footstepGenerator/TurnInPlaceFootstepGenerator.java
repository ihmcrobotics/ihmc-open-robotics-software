package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.TurningOverheadPath;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TurnInPlaceFootstepGenerator extends AbstractSimpleParametersFootstepGenerator
{
   private TurningOverheadPath footstepPath;
   private FrameOrientation2D endOrientation;

   public TurnInPlaceFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FrameOrientation2D pathYaw, PathTypeStepParameters pathType)
   {
      super(feet, soleFrames, pathType);
      this.endOrientation = pathYaw;
   }

   protected void initialize(FramePose2D startPose)
   {
      setFootstepPath(new TurningOverheadPath(startPose, endOrientation));
      footstepCounter = new FootstepCounterForSingleTurnPath();
   }

   protected void initializeFootstepCounter(double openingAngle, double closingAngle, double stepLength, boolean hipExtensionFirst,
           boolean willSetFirstStraightStepToFarSideIfNoTurns, double standardFootWidth)
   {
      ((FootstepCounterForSingleTurnPath) footstepCounter).initialize(openingAngle, closingAngle, getSignedInitialTurnDirection(), hipExtensionFirst, initialDeltaFeetYaw);
   }

   protected double getSignedInitialTurnDirection()
   {
      startPose.checkReferenceFrameMatch(endOrientation);
      FrameOrientation2D startOrientation = new FrameOrientation2D(startPose.getOrientation());
      double deltaYaw = endOrientation.difference(startOrientation);

      return deltaYaw;
   }

   public void setFootstepPath(TurningOverheadPath turningOverheadPath)
   {
      this.footstepPath = turningOverheadPath;
   }

   protected TurningOverheadPath getPath()
   {
      return footstepPath;
   }

   public double getTurningAmount()
   {
      return getSignedInitialTurnDirection();
   }
   
   public boolean hasDisplacement()
   {
      return (Math.abs(getTurningAmount()) > noTranslationTolerance);
   }
   
   public boolean turnStepsTaken() {
      return ((FootstepCounterForSingleTurnPath) footstepCounter).turnStepsTaken();
   }

   public double getOvershootStepFraction() {
      return ((FootstepCounterForSingleTurnPath) footstepCounter).getOvershootStepFraction();
   }

}
