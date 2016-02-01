package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.TurningOverheadPath;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TurnInPlaceFootstepGenerator extends AbstractSimpleParametersFootstepGenerator
{
   private TurningOverheadPath footstepPath;
   private FrameOrientation2d endOrientation;

   public TurnInPlaceFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, FrameOrientation2d pathYaw, PathTypeStepParameters pathType)
   {
      super(feet, soleFrames, pathType);
      this.endOrientation = pathYaw;
   }

   protected void initialize(FramePose2d startPose)
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
      FrameOrientation2d startOrientation = new FrameOrientation2d();
      startPose.getOrientation(startOrientation);
      double deltaYaw = endOrientation.sub(startOrientation);

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
