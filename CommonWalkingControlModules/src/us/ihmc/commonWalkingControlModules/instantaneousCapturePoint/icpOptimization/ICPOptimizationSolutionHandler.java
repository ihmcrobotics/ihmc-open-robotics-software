package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.StateMultiplierCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPOptimizationSolutionHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint2d actualEndingCornerPoint;
   private final YoFramePoint2d referenceICP;
   private final YoFrameVector2d referenceICPVelocity;
   private final YoFramePoint2d referenceCMP;

   private final YoFramePoint2d nominalEndingCornerPoint;
   private final YoFramePoint2d nominalReferenceICP;
   private final YoFrameVector2d nominalReferenceICPVelocity;
   private final YoFramePoint2d nominalReferenceCMP;

   private final DoubleYoVariable footstepDeadband;
   private final DoubleYoVariable footstepSolutionResolution;

   private final BooleanYoVariable footstepWasAdjusted;
   private final YoFrameVector2d footstepAdjustment;

   private final DoubleYoVariable costToGo;
   private final DoubleYoVariable footstepCostToGo;
   private final DoubleYoVariable feedbackCostToGo;
   private final DoubleYoVariable dynamicRelaxationCostToGo;

   private final StateMultiplierCalculator stateMultiplierCalculator;

   private final boolean debug;
   private final String yoNamePrefix;

   public ICPOptimizationSolutionHandler(ICPOptimizationParameters icpOptimizationParameters, StateMultiplierCalculator stateMultiplierCalculator,
         boolean visualize, boolean debug, String yoNamePrefix, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.stateMultiplierCalculator = stateMultiplierCalculator;
      this.debug = debug;
      this.yoNamePrefix = yoNamePrefix;

      actualEndingCornerPoint = new YoFramePoint2d(yoNamePrefix + "ActualEndingCornerPoint", worldFrame, registry);

      referenceICP = new YoFramePoint2d(yoNamePrefix + "ReferenceICP", worldFrame, registry);
      referenceICPVelocity = new YoFrameVector2d(yoNamePrefix + "ReferenceICPVelocity", worldFrame, registry);
      referenceCMP = new YoFramePoint2d(yoNamePrefix + "ReferenceCMP", worldFrame, registry);

      if (debug)
      {
         nominalEndingCornerPoint = new YoFramePoint2d(yoNamePrefix + "NominalEndingCornerPoint", worldFrame, registry);
         nominalReferenceICP = new YoFramePoint2d(yoNamePrefix + "NominalReferenceICP", worldFrame, registry);
         nominalReferenceICPVelocity = new YoFrameVector2d(yoNamePrefix + "NominalReferenceICPVelocity", worldFrame, registry);
         nominalReferenceCMP = new YoFramePoint2d(yoNamePrefix + "NominalReferenceCMP", worldFrame, registry);

         costToGo = new DoubleYoVariable(yoNamePrefix + "CostToGo", registry);
         footstepCostToGo = new DoubleYoVariable(yoNamePrefix + "FootstepCostToGo", registry);
         feedbackCostToGo = new DoubleYoVariable(yoNamePrefix + "FeedbackCostToGo", registry);
         dynamicRelaxationCostToGo = new DoubleYoVariable(yoNamePrefix + "DynamicRelaxationCostToGo", registry);
      }
      else
      {
         nominalEndingCornerPoint = null;
         nominalReferenceICP = null;
         nominalReferenceICPVelocity = null;
         nominalReferenceCMP = null;

         costToGo = null;
         footstepCostToGo = null;
         feedbackCostToGo = null;
         dynamicRelaxationCostToGo = null;
      }

      footstepDeadband = new DoubleYoVariable(yoNamePrefix + "FootstepDeadband", registry);
      footstepSolutionResolution = new DoubleYoVariable(yoNamePrefix + "FootstepSolutionResolution", registry);

      footstepWasAdjusted = new BooleanYoVariable(yoNamePrefix + "FootstepWasAdjusted", registry);
      footstepAdjustment = new YoFrameVector2d(yoNamePrefix + "FootstepAdjustment", worldFrame, registry);

      footstepDeadband.set(icpOptimizationParameters.getAdjustmentDeadband());
      footstepSolutionResolution.set(icpOptimizationParameters.getFootstepSolutionResolution());

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry, visualize);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition actualEndingCornerPoint = new YoGraphicPosition(yoNamePrefix + "ActualEndingCornerPoint", this.actualEndingCornerPoint, 0.005, YoAppearance.Aquamarine(),
            GraphicType.SOLID_BALL);

      if (debug)
      {
         YoGraphicPosition nominalReferenceICP = new YoGraphicPosition(yoNamePrefix + "NominalReferenceICP", this.nominalReferenceICP, 0.01, YoAppearance.LightYellow(),
               GraphicType.BALL);
         YoGraphicPosition nominalEndingCornerPoint = new YoGraphicPosition(yoNamePrefix + "NominalEndingCornerPoint", this.nominalEndingCornerPoint, 0.01, YoAppearance.Green(),
               GraphicType.SOLID_BALL);
         yoGraphicsList.add(nominalReferenceICP);
         yoGraphicsList.add(nominalEndingCornerPoint);
         artifactList.add(nominalReferenceICP.createArtifact());
         artifactList.add(nominalEndingCornerPoint.createArtifact());
      }

      yoGraphicsList.add(actualEndingCornerPoint);
      artifactList.add(actualEndingCornerPoint.createArtifact());

      yoGraphicsList.setVisible(visualize);
      artifactList.setVisible(visualize);

      YoGraphicPosition referenceICP = new YoGraphicPosition(yoNamePrefix + "ReferenceICP", this.referenceICP, 0.005, YoAppearance.Yellow(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition referenceCMP = new YoGraphicPosition(yoNamePrefix + "ReferenceCMP", this.referenceCMP, 0.005, YoAppearance.Beige(), GraphicType.BALL_WITH_CROSS);

      String name = "ICPOptimization";
      yoGraphicsListRegistry.registerArtifact(name, referenceICP.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, referenceCMP.createArtifact());
      yoGraphicsListRegistry.registerYoGraphic(name, referenceICP);
      yoGraphicsListRegistry.registerYoGraphic(name, referenceCMP);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void updateCostsToGo(ICPOptimizationSolver solver)
   {
      if (debug)
      {
         costToGo.set(solver.getCostToGo());
         footstepCostToGo.set(solver.getFootstepCostToGo());
         feedbackCostToGo.set(solver.getFeedbackCostToGo());
         dynamicRelaxationCostToGo.set(solver.getDynamicRelaxationCostToGo());
      }
   }

   private final FramePoint2d locationSolution = new FramePoint2d();
   private final FramePoint2d clippedLocationSolution = new FramePoint2d();
   private final FramePoint2d upcomingFootstepLocation = new FramePoint2d();

   public void extractFootstepSolutions(ArrayList<YoFramePoint2d> footstepSolutionsToPack, ArrayList<FramePoint2d> unclippedFootstepSolutionsToPack,
         ArrayList<YoFramePoint2d> referenceFootstepLocations, ArrayList<Footstep> upcomingFootsteps, int numberOfFootstepsToConsider, ICPOptimizationSolver solver)
   {
      boolean firstStepAdjusted = false;
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         solver.getFootstepSolutionLocation(i, locationSolution);

         upcomingFootsteps.get(i).getPosition2d(upcomingFootstepLocation);
         ReferenceFrame deadbandFrame = upcomingFootsteps.get(i).getSoleReferenceFrame();

         FramePoint2d referenceFootstepLocation = referenceFootstepLocations.get(i).getFrameTuple2d();
         clippedLocationSolution.set(locationSolution);
         boolean footstepWasAdjusted = applyLocationDeadband(clippedLocationSolution, upcomingFootstepLocation, referenceFootstepLocations.get(i).getFrameTuple2d(), deadbandFrame,
               footstepDeadband.getDoubleValue(), footstepSolutionResolution.getDoubleValue());

         if (i == 0)
         {
            firstStepAdjusted = footstepWasAdjusted;
            footstepAdjustment.set(locationSolution);
            footstepAdjustment.sub(referenceFootstepLocation);
         }

         footstepSolutionsToPack.get(i).set(clippedLocationSolution);
         unclippedFootstepSolutionsToPack.get(i).set(locationSolution);
      }

      this.footstepWasAdjusted.set(firstStepAdjusted);
   }

   private final FramePoint solutionLocation = new FramePoint();
   private final FramePoint referenceLocation = new FramePoint();
   private final FramePoint previousLocation = new FramePoint();
   private final FrameVector solutionAdjustment = new FrameVector();
   private final FrameVector adjustmentFromPrevious = new FrameVector();

   private final FrameVector tmpVector = new FrameVector();

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
      adjustmentFromPrevious.setToZero(worldFrame);

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
         tmpVector.setIncludingFrame(solutionAdjustment);
         tmpVector.normalize();
         tmpVector.scale(deadband);

         solutionLocation.changeFrame(deadbandFrame);
         solutionLocation.sub(tmpVector);
      }

      solutionLocation.changeFrame(worldFrame);
      adjustmentFromPrevious.set(solutionLocation);
      adjustmentFromPrevious.sub(previousLocation);
      adjustmentFromPrevious.changeFrame(deadbandFrame);

      if (adjustmentFromPrevious.length() < deadbandResolution)
         solutionLocation.set(previousLocation);
      else
         wasAdjusted = true;

      solutionLocation.changeFrame(solutionLocationToPack.getReferenceFrame());
      solutionLocationToPack.setByProjectionOntoXYPlane(solutionLocation);

      return wasAdjusted;
   }

   private final FramePoint2d tmpEndPoint = new FramePoint2d();
   private final FramePoint2d tmpReferencePoint = new FramePoint2d();
   private final FramePoint2d tmpCMP = new FramePoint2d();
   private final FrameVector2d tmpReferenceVelocity = new FrameVector2d();
   private final FramePoint2d finalICP2d = new FramePoint2d();

   public void computeReferenceFromSolutions(ArrayList<FramePoint2d> footstepSolutions, ICPOptimizationInputHandler inputHandler,
         YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      computeReferenceFromSolutions(footstepSolutions, inputHandler.getEntryOffsets(), inputHandler.getExitOffsets(), inputHandler.getStanceEntryCMP(),
            inputHandler.getStanceExitCMP(), inputHandler.getFinalICP(), beginningOfStateICP, beginningOfStateICPVelocity, omega0, numberOfFootstepsToConsider);
   }

   public void computeReferenceFromSolutions(ArrayList<FramePoint2d> footstepSolutions, ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets,
         FramePoint2d stanceEntryCMP, FramePoint2d stanceExitCMP, FramePoint finalICP, YoFramePoint2d beginningOfStateICP,
         YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      finalICP.getFrameTuple2d(finalICP2d);
      stateMultiplierCalculator.reconstructICPCornerPoint(tmpEndPoint, finalICP2d, footstepSolutions, entryOffsets, exitOffsets,
            numberOfFootstepsToConsider);
      stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tmpReferenceVelocity, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
                  beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega0, tmpCMP);

      actualEndingCornerPoint.set(tmpEndPoint);
      referenceICP.set(tmpReferencePoint);
      referenceICPVelocity.set(tmpReferenceVelocity);
      referenceCMP.set(tmpCMP);
   }

   public void yoComputeReferenceFromSolutions(ArrayList<YoFramePoint2d> footstepSolutions, ICPOptimizationInputHandler inputHandler,
         YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      yoComputeReferenceFromSolutions(footstepSolutions, inputHandler.getEntryOffsets(), inputHandler.getExitOffsets(), inputHandler.getStanceEntryCMP(),
            inputHandler.getStanceExitCMP(), inputHandler.getFinalICP(), beginningOfStateICP, beginningOfStateICPVelocity, omega0, numberOfFootstepsToConsider);
   }

   public void yoComputeReferenceFromSolutions(ArrayList<YoFramePoint2d> footstepSolutions, ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets,
         FramePoint2d stanceEntryCMP, FramePoint2d stanceExitCMP, FramePoint finalICP, YoFramePoint2d beginningOfStateICP,
         YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      finalICP.getFrameTuple2d(finalICP2d);
      stateMultiplierCalculator.yoReconstructICPCornerPoint(tmpEndPoint, finalICP2d, footstepSolutions, entryOffsets, exitOffsets,
            numberOfFootstepsToConsider);
      stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tmpReferenceVelocity, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
            beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega0, tmpCMP);

      actualEndingCornerPoint.set(tmpEndPoint);
      referenceICP.set(tmpReferencePoint);
      referenceICPVelocity.set(tmpReferenceVelocity);
      referenceCMP.set(tmpCMP);
   }

   public void computeNominalValues(ArrayList<YoFramePoint2d> upcomingFootstepLocations, ICPOptimizationInputHandler inputHandler,
         YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      if (debug)
      {
         computeNominalValues(upcomingFootstepLocations, inputHandler.getEntryOffsets(), inputHandler.getExitOffsets(), inputHandler.getStanceEntryCMP(),
               inputHandler.getStanceExitCMP(), inputHandler.getFinalICP(), beginningOfStateICP, beginningOfStateICPVelocity, omega0,
               numberOfFootstepsToConsider);
      }
   }

   public void computeNominalValues(ArrayList<YoFramePoint2d> upcomingFootstepLocations, ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets,
         FramePoint2d stanceEntryCMP, FramePoint2d stanceExitCMP, FramePoint finalICP, YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity,
         double omega0, int numberOfFootstepsToConsider)
   {
      if (debug)
      {
         finalICP.getFrameTuple2d(finalICP2d);
         stateMultiplierCalculator
               .yoReconstructICPCornerPoint(tmpEndPoint, finalICP2d, upcomingFootstepLocations, entryOffsets, exitOffsets,
                     numberOfFootstepsToConsider);
         stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tmpReferenceVelocity, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
               beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());

         CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega0, tmpCMP);

         nominalEndingCornerPoint.set(tmpEndPoint);
         nominalReferenceICP.set(tmpReferencePoint);
         nominalReferenceICPVelocity.set(tmpReferenceVelocity);
         nominalReferenceCMP.set(tmpCMP);
      }
   }

   public void setValuesForFeedbackOnly(FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, double omega0)
   {
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega0, tmpCMP);

      referenceICP.set(desiredICP);
      referenceICPVelocity.set(desiredICPVelocity);
      referenceCMP.set(tmpCMP);
      if (debug)
      {
         nominalReferenceICP.set(desiredICP);
         nominalReferenceICPVelocity.set(desiredICPVelocity);
         nominalReferenceCMP.set(tmpCMP);
      }
   }

   public FramePoint2d getControllerReferenceICP()
   {
      return referenceICP.getFrameTuple2d();
   }

   public FrameVector2d getControllerReferenceICPVelocity()
   {
      return referenceICPVelocity.getFrameTuple2d();
   }

   public void getControllerReferenceCMP(FramePoint2d framePointToPack)
   {
      referenceCMP.getFrameTuple2d(framePointToPack);
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

