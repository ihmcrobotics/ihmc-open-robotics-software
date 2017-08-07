package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPQPOptimizationSolver;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.AbstractSimpleICPOptimizationController.yoNamePrefix;

public class SimpleICPOptimizationSolutionHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble footstepDeadband;
   private final YoDouble footstepSolutionResolution;

   private final YoBoolean footstepWasAdjusted;
   private final YoFrameVector2d footstepAdjustment;
   private final YoFrameVector2d clippedFootstepAdjustment;

   private final YoDouble residualCostToGo;
   private final YoDouble costToGo;
   private final YoDouble footstepCostToGo;
   private final YoDouble feedbackCostToGo;
   private final YoDouble dynamicRelaxationCostToGo;
   private final YoDouble angularMomentumMinimizationCostToGo;

   private final YoFramePoint2d adjustedICPReferenceLocation;

   private final boolean debug;

   private final FramePoint2d locationSolution = new FramePoint2d();
   private final FramePoint2d previousLocationSolution = new FramePoint2d();
   private final FramePoint2d clippedLocationSolution = new FramePoint2d();

   private final FramePoint solutionLocation = new FramePoint();
   private final FramePoint referenceLocation = new FramePoint();
   private final FramePoint previousLocation = new FramePoint();
   private final FrameVector solutionAdjustment = new FrameVector();

   private final FrameVector tempVector = new FrameVector();

   public SimpleICPOptimizationSolutionHandler(ICPOptimizationParameters icpOptimizationParameters, boolean debug, String yoNamePrefix, YoVariableRegistry registry)
   {
      this.debug = debug;

      if (debug)
      {
         residualCostToGo = new YoDouble(yoNamePrefix + "ResidualCostToGo", registry);
         costToGo = new YoDouble(yoNamePrefix + "CostToGo", registry);
         footstepCostToGo = new YoDouble(yoNamePrefix + "FootstepCostToGo", registry);
         feedbackCostToGo = new YoDouble(yoNamePrefix + "FeedbackCostToGo", registry);
         dynamicRelaxationCostToGo = new YoDouble(yoNamePrefix + "DynamicRelaxationCostToGo", registry);
         angularMomentumMinimizationCostToGo = new YoDouble(yoNamePrefix + "AngularMomentumMinimizationCostToGo", registry);
      }
      else
      {
         residualCostToGo = null;
         costToGo = null;
         footstepCostToGo = null;
         feedbackCostToGo = null;
         dynamicRelaxationCostToGo = null;
         angularMomentumMinimizationCostToGo = null;
      }

      footstepDeadband = new YoDouble(yoNamePrefix + "FootstepDeadband", registry);
      footstepSolutionResolution = new YoDouble(yoNamePrefix + "FootstepSolutionResolution", registry);

      footstepWasAdjusted = new YoBoolean(yoNamePrefix + "FootstepWasAdjusted", registry);
      footstepAdjustment = new YoFrameVector2d(yoNamePrefix + "FootstepAdjustment", worldFrame, registry);
      clippedFootstepAdjustment = new YoFrameVector2d(yoNamePrefix + "ClippedFootstepAdjustment", worldFrame, registry);

      adjustedICPReferenceLocation = new YoFramePoint2d(yoNamePrefix + "AdjustedICPReferenceLocation", worldFrame, registry);

      footstepDeadband.set(icpOptimizationParameters.getAdjustmentDeadband());
      footstepSolutionResolution.set(icpOptimizationParameters.getFootstepSolutionResolution());
   }

   public void setupVisualizers(ArtifactList artifactList)
   {
      YoGraphicPosition adjustedICP = new YoGraphicPosition(yoNamePrefix + "AdjustedICPReferencedLocation", adjustedICPReferenceLocation, 0.01, YoAppearance.LightYellow(),
                                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      artifactList.add(adjustedICP.createArtifact());
   }

   public void updateCostsToGo(SimpleICPOptimizationQPSolver solver)
   {
      if (debug)
      {
         residualCostToGo.set(solver.getCostToGo());
         costToGo.set(solver.getCostToGo());
         footstepCostToGo.set(solver.getFootstepCostToGo());
         feedbackCostToGo.set(solver.getFeedbackCostToGo());
         angularMomentumMinimizationCostToGo.set(solver.getAngularMomentumMinimizationCostToGo());
      }
   }

   private final FramePoint2d referenceFootstepLocation = new FramePoint2d();
   public void extractFootstepSolutions(ArrayList<YoFramePoint2d> footstepSolutionsToPack, ArrayList<FramePoint2d> unclippedFootstepSolutionsToPack,
         ArrayList<Footstep> upcomingFootsteps, int numberOfFootstepsToConsider, SimpleICPOptimizationQPSolver solver)
   {
      boolean firstStepAdjusted = false;
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         solver.getFootstepSolutionLocation(i, locationSolution);

         ReferenceFrame deadbandFrame = upcomingFootsteps.get(i).getSoleReferenceFrame();

         //referenceFootstepLocations.get(i).getFrameTuple2d(referenceFootstepLocation);
         upcomingFootsteps.get(i).getPosition2d(referenceFootstepLocation);
         footstepSolutionsToPack.get(i).getFrameTuple2d(previousLocationSolution);
         clippedLocationSolution.set(locationSolution);
         boolean footstepWasAdjusted = applyLocationDeadband(clippedLocationSolution, previousLocationSolution, referenceFootstepLocation,
               deadbandFrame, footstepDeadband.getDoubleValue(), footstepSolutionResolution.getDoubleValue());

         if (i == 0)
         {
            firstStepAdjusted = footstepWasAdjusted;
            footstepAdjustment.set(locationSolution);
            footstepAdjustment.sub(referenceFootstepLocation);
            clippedFootstepAdjustment.set(clippedLocationSolution);
            clippedFootstepAdjustment.sub(referenceFootstepLocation);
         }

         footstepSolutionsToPack.get(i).set(clippedLocationSolution);
         unclippedFootstepSolutionsToPack.get(i).set(locationSolution);
      }

      this.footstepWasAdjusted.set(firstStepAdjusted);
   }

   public void zeroAdjustment()
   {
      footstepAdjustment.setToZero();
      clippedFootstepAdjustment.setToZero();
      footstepWasAdjusted.set(false);
   }

   public void updateVisualizers(FramePoint2d desiredICP, double footstepMultiplier)
   {
      adjustedICPReferenceLocation.set(clippedFootstepAdjustment);
      adjustedICPReferenceLocation.scale(footstepMultiplier);
      adjustedICPReferenceLocation.add(desiredICP);
   }

   private boolean applyLocationDeadband(FramePoint2d solutionLocationToPack, FramePoint2d currentSolutionLocation, FramePoint2d referenceLocation2d,
         ReferenceFrame deadbandFrame, double deadband, double deadbandResolution)
   {
      solutionLocation.setXYIncludingFrame(solutionLocationToPack);
      referenceLocation.setXYIncludingFrame(referenceLocation2d);
      previousLocation.setXYIncludingFrame(currentSolutionLocation);

      solutionLocation.changeFrame(worldFrame);
      referenceLocation.changeFrame(worldFrame);
      previousLocation.changeFrame(worldFrame);

      solutionAdjustment.setToZero(worldFrame);
      solutionAdjustment.set(solutionLocation);
      solutionAdjustment.sub(referenceLocation);

      solutionAdjustment.changeFrame(deadbandFrame);

      boolean wasAdjusted = false;

      if (solutionAdjustment.length() < deadband)
      {
         solutionLocation.set(referenceLocation);
      }
      else
      {
         tempVector.setIncludingFrame(solutionAdjustment);
         tempVector.normalize();
         tempVector.scale(deadband);

         solutionLocation.changeFrame(deadbandFrame);
         solutionLocation.sub(tempVector);
      }

      solutionLocation.changeFrame(worldFrame);
      tempVector.setToZero(worldFrame);
      tempVector.set(solutionLocation);
      tempVector.sub(previousLocation);
      tempVector.changeFrame(deadbandFrame);

      if (tempVector.length() < deadbandResolution)
         solutionLocation.set(previousLocation);
      else
         wasAdjusted = true;

      solutionLocation.changeFrame(solutionLocationToPack.getReferenceFrame());
      solutionLocationToPack.setByProjectionOntoXYPlane(solutionLocation);

      return wasAdjusted;
   }

   public boolean wasFootstepAdjusted()
   {
      return footstepWasAdjusted.getBooleanValue();
   }

   public FrameVector2d getFootstepAdjustment()
   {
      return footstepAdjustment.getFrameTuple2d();
   }
}

