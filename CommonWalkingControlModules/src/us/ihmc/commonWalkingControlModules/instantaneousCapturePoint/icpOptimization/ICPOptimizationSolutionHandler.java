package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.NewStateMultiplierCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.FootstepRecursionMultiplierCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
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
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;

public class ICPOptimizationSolutionHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint2d actualEndOfStateICP;
   private final YoFramePoint2d controllerReferenceICP;
   private final YoFrameVector2d controllerReferenceICPVelocity;
   private final YoFramePoint2d controllerReferenceCMP;

   private final YoFramePoint2d nominalEndOfStateICP;
   private final YoFramePoint2d nominalReferenceICP;
   private final YoFrameVector2d nominalReferenceICPVelocity;
   private final YoFramePoint2d nominalReferenceCMP;

   private final DoubleYoVariable footstepDeadband;
   private final DoubleYoVariable footstepSolutionResolution;

   private final BooleanYoVariable footstepWasAdjusted;
   private final YoFrameVector2d footstepAdjustment;

   private final DoubleYoVariable controllerCostToGo;
   private final DoubleYoVariable controllerFootstepCostToGo;
   private final DoubleYoVariable controllerFeedbackCostToGo;
   private final DoubleYoVariable controllerDynamicRelaxationCostToGo;

   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator;
   private final NewStateMultiplierCalculator stateMultiplierCalculator;
   private final boolean useNewMultiplierCalculator;

   public ICPOptimizationSolutionHandler(ICPOptimizationParameters icpOptimizationParameters, FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator,
         NewStateMultiplierCalculator stateMultiplierCalculator, boolean visualize, boolean useNewMultiplierCalculator,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.footstepRecursionMultiplierCalculator = footstepRecursionMultiplierCalculator;
      this.stateMultiplierCalculator = stateMultiplierCalculator;
      this.useNewMultiplierCalculator = useNewMultiplierCalculator;

      actualEndOfStateICP = new YoFramePoint2d("actualEndOfStateICP", worldFrame, registry);

      controllerReferenceICP = new YoFramePoint2d("controllerReferenceICP", worldFrame, registry);
      controllerReferenceICPVelocity = new YoFrameVector2d("controllerReferenceICPVelocity", worldFrame, registry);
      controllerReferenceCMP = new YoFramePoint2d("controllerReferenceCMP", worldFrame, registry);

      nominalEndOfStateICP = new YoFramePoint2d("nominalEndOfStateICP", worldFrame, registry);
      nominalReferenceICP = new YoFramePoint2d("nominalReferenceICP", worldFrame, registry);
      nominalReferenceICPVelocity = new YoFrameVector2d("nominalReferenceICPVelocity", worldFrame, registry);
      nominalReferenceCMP = new YoFramePoint2d("nominalReferenceCMP", worldFrame, registry);

      controllerCostToGo = new DoubleYoVariable("costToGo", registry);
      controllerFootstepCostToGo = new DoubleYoVariable("footstepCostToGo", registry);
      controllerFeedbackCostToGo = new DoubleYoVariable("feedbackCostToGo", registry);
      controllerDynamicRelaxationCostToGo = new DoubleYoVariable("dynamicRelaxationCostToGo", registry);

      footstepDeadband = new DoubleYoVariable("footstepDeadband", registry);
      footstepSolutionResolution = new DoubleYoVariable("footstepSolutionResolution", registry);

      footstepWasAdjusted = new BooleanYoVariable("footstepWasAdjusted", registry);
      footstepAdjustment = new YoFrameVector2d("footstepAdjustment", worldFrame, registry);

      footstepDeadband.set(icpOptimizationParameters.getAdjustmentDeadband());
      footstepSolutionResolution.set(icpOptimizationParameters.getFootstepSolutionResolution());

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry, visualize);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition actualEndOfStateICP = new YoGraphicPosition("actualEndOfStateICP", this.actualEndOfStateICP, 0.005, YoAppearance.Aquamarine(),
            GraphicType.SOLID_BALL);

      YoGraphicPosition nominalReferenceICP = new YoGraphicPosition("nominalReferenceICP", this.nominalReferenceICP, 0.01, YoAppearance.LightYellow(),
            GraphicType.BALL);
      YoGraphicPosition nominalEndOfStateICP = new YoGraphicPosition("nominalEndOfStateICP", this.nominalEndOfStateICP, 0.01, YoAppearance.Green(),
            GraphicType.SOLID_BALL);

      yoGraphicsList.add(actualEndOfStateICP);
      yoGraphicsList.add(nominalReferenceICP);
      yoGraphicsList.add(nominalEndOfStateICP);
      artifactList.add(actualEndOfStateICP.createArtifact());
      artifactList.add(nominalReferenceICP.createArtifact());
      artifactList.add(nominalEndOfStateICP.createArtifact());

      yoGraphicsList.setVisible(visualize);
      artifactList.setVisible(visualize);

      YoGraphicPosition referenceICP = new YoGraphicPosition("controllerReferenceICP", controllerReferenceICP, 0.005, YoAppearance.Yellow(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition referenceCMP = new YoGraphicPosition("controllerReferenceCMP", controllerReferenceCMP, 0.005, YoAppearance.Beige(), GraphicType.BALL_WITH_CROSS);

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
      controllerCostToGo.set(solver.getCostToGo());
      controllerFootstepCostToGo.set(solver.getFootstepCostToGo());
      controllerFeedbackCostToGo.set(solver.getFeedbackCostToGo());
      controllerDynamicRelaxationCostToGo.set(solver.getDynamicRelaxationCostToGo());
   }

   private final FramePoint2d locationSolution = new FramePoint2d();
   private final FramePoint2d upcomingFootstepLocation = new FramePoint2d();

   public void extractFootstepSolutions(ArrayList<YoFramePoint2d> footstepSolutionsToPack, ArrayList<YoFramePoint2d> referenceFootstepLocations,
         ArrayList<Footstep> upcomingFootsteps, int numberOfFootstepsToConsider,
         ICPOptimizationSolver solver)
   {
      boolean firstStepAdjusted = false;
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         solver.getFootstepSolutionLocation(i, locationSolution);

         upcomingFootsteps.get(i).getPosition2d(upcomingFootstepLocation);
         ReferenceFrame deadbandFrame = upcomingFootsteps.get(i).getSoleReferenceFrame();

         FramePoint2d referenceFootstepLocation = referenceFootstepLocations.get(i).getFrameTuple2d();
         boolean footstepWasAdjusted = applyLocationDeadband(locationSolution, upcomingFootstepLocation, referenceFootstepLocation, deadbandFrame,
               footstepDeadband.getDoubleValue(), footstepSolutionResolution.getDoubleValue());

         if (i == 0)
         {
            firstStepAdjusted = footstepWasAdjusted;
            footstepAdjustment.set(locationSolution);
            footstepAdjustment.sub(referenceFootstepLocation);
         }

         footstepSolutionsToPack.get(i).set(locationSolution);
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

   public void computeReferenceFromSolutions(ArrayList<YoFramePoint2d> footstepSolutions, ICPOptimizationInputHandler inputHandler,
         YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      computeReferenceFromSolutions(footstepSolutions, inputHandler.getEntryOffsets(), inputHandler.getExitOffsets(), inputHandler.getPreviousStanceExitCMP(),
            inputHandler.getStanceEntryCMP(), inputHandler.getStanceExitCMP(), inputHandler.getFinalICP(), beginningOfStateICP, beginningOfStateICPVelocity,
            omega0, numberOfFootstepsToConsider);
   }

   public void computeReferenceFromSolutions(ArrayList<YoFramePoint2d> footstepSolutions, ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets,
         FramePoint2d previousStanceExitCMP, FramePoint2d stanceEntryCMP, FramePoint2d stanceExitCMP, FramePoint finalICP, YoFramePoint2d beginningOfStateICP,
         YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      finalICP.getFrameTuple2d(finalICP2d);
      if (useNewMultiplierCalculator)
      {
         stateMultiplierCalculator.reconstructICPCornerPoint(tmpEndPoint, finalICP2d, footstepSolutions, entryOffsets, exitOffsets, stanceEntryCMP, stanceExitCMP,
               numberOfFootstepsToConsider);
         stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tmpReferenceVelocity, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
                     beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());
      }
      else
      {
         footstepRecursionMultiplierCalculator
               .computeICPPoints(finalICP2d, footstepSolutions, entryOffsets, exitOffsets, previousStanceExitCMP, stanceEntryCMP, stanceExitCMP,
                     beginningOfStateICP.getFrameTuple2d(), numberOfFootstepsToConsider, tmpEndPoint, tmpReferencePoint, tmpReferenceVelocity);
      }

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega0, tmpCMP);

      actualEndOfStateICP.set(tmpEndPoint);
      controllerReferenceICP.set(tmpReferencePoint);
      controllerReferenceICPVelocity.set(tmpReferenceVelocity);
      controllerReferenceCMP.set(tmpCMP);
   }

   public void computeNominalValues(ArrayList<YoFramePoint2d> upcomingFootstepLocations, ICPOptimizationInputHandler inputHandler,
         YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      computeNominalValues(upcomingFootstepLocations, inputHandler.getEntryOffsets(), inputHandler.getExitOffsets(), inputHandler.getPreviousStanceExitCMP(),
            inputHandler.getStanceEntryCMP(), inputHandler.getStanceExitCMP(), inputHandler.getFinalICP(), beginningOfStateICP, beginningOfStateICPVelocity,
            omega0, numberOfFootstepsToConsider);
   }

   public void computeNominalValues(ArrayList<YoFramePoint2d> upcomingFootstepLocations, ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets,
         FramePoint2d previousStanceExitCMP, FramePoint2d stanceEntryCMP, FramePoint2d stanceExitCMP, FramePoint finalICP, YoFramePoint2d beginningOfStateICP,
         YoFrameVector2d beginningOfStateICPVelocity, double omega0, int numberOfFootstepsToConsider)
   {
      finalICP.getFrameTuple2d(finalICP2d);
      if (useNewMultiplierCalculator)
      {
         stateMultiplierCalculator.reconstructICPCornerPoint(tmpEndPoint, finalICP2d, upcomingFootstepLocations, entryOffsets, exitOffsets, stanceEntryCMP, stanceExitCMP, numberOfFootstepsToConsider);
         stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tmpReferenceVelocity, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
               beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());
      }
      else
      {
         footstepRecursionMultiplierCalculator
               .computeICPPoints(finalICP2d, upcomingFootstepLocations, entryOffsets, exitOffsets, previousStanceExitCMP, stanceEntryCMP, stanceExitCMP,
                     beginningOfStateICP.getFrameTuple2d(), numberOfFootstepsToConsider, tmpEndPoint, tmpReferencePoint, tmpReferenceVelocity);
      }

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega0, tmpCMP);

      nominalEndOfStateICP.set(tmpEndPoint);
      nominalReferenceICP.set(tmpReferencePoint);
      nominalReferenceICPVelocity.set(tmpReferenceVelocity);
      nominalReferenceCMP.set(tmpCMP);
   }

   public void setValuesForFeedbackOnly(FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, double omega0)
   {
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega0, tmpCMP);

      controllerReferenceICP.set(desiredICP);
      controllerReferenceICPVelocity.set(desiredICPVelocity);
      controllerReferenceCMP.set(tmpCMP);
      nominalReferenceICP.set(desiredICP);
      nominalReferenceICPVelocity.set(desiredICPVelocity);
      nominalReferenceCMP.set(tmpCMP);
   }

   public FramePoint2d getControllerReferenceICP()
   {
      return controllerReferenceICP.getFrameTuple2d();
   }

   public FrameVector2d getControllerReferenceICPVelocity()
   {
      return controllerReferenceICPVelocity.getFrameTuple2d();
   }

   public void getControllerReferenceCMP(FramePoint2d framePointToPack)
   {
      controllerReferenceCMP.getFrameTuple2d(framePointToPack);
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

