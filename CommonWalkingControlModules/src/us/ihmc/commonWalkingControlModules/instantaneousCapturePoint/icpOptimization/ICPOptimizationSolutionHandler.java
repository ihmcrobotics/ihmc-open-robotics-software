package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.StateMultiplierCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

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

   private final YoDouble footstepDeadband;
   private final YoDouble footstepSolutionResolution;

   private final YoBoolean footstepWasAdjusted;
   private final YoFrameVector2d footstepAdjustment;

   private final YoDouble residualCostToGo;
   private final YoDouble costToGo;
   private final YoDouble footstepCostToGo;
   private final YoDouble feedbackCostToGo;
   private final YoDouble dynamicRelaxationCostToGo;
   private final YoDouble angularMomentumMinimizationCostToGo;

   private final boolean debug;
   private final String yoNamePrefix;
   
   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole;

   private final FramePoint2d locationSolution = new FramePoint2d();
   private final FramePoint2d clippedLocationSolution = new FramePoint2d();
   private final FramePoint2d upcomingFootstepLocation = new FramePoint2d();

   private final FramePoint2d tmpEndPoint = new FramePoint2d();
   private final FramePoint2d tmpReferencePoint = new FramePoint2d();

   private final FramePoint solutionLocation = new FramePoint();
   private final FramePoint referenceLocation = new FramePoint();
   private final FramePoint previousLocation = new FramePoint();
   private final FrameVector solutionAdjustment = new FrameVector();

   private final FrameVector tempVector = new FrameVector();
   private final FramePoint2d tempPoint2d = new FramePoint2d();
   private final FrameVector2d tempVector2d = new FrameVector2d();

   public ICPOptimizationSolutionHandler(ICPOptimizationParameters icpOptimizationParameters, SideDependentList<RigidBodyTransform> transformsFromAnkleToSole,
         boolean visualize, boolean debug, String yoNamePrefix, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.debug = debug;
      this.yoNamePrefix = yoNamePrefix;
      this.transformsFromAnkleToSole = transformsFromAnkleToSole;

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

         residualCostToGo = new YoDouble(yoNamePrefix + "ResidualCostToGo", registry);
         costToGo = new YoDouble(yoNamePrefix + "CostToGo", registry);
         footstepCostToGo = new YoDouble(yoNamePrefix + "FootstepCostToGo", registry);
         feedbackCostToGo = new YoDouble(yoNamePrefix + "FeedbackCostToGo", registry);
         dynamicRelaxationCostToGo = new YoDouble(yoNamePrefix + "DynamicRelaxationCostToGo", registry);
         angularMomentumMinimizationCostToGo = new YoDouble(yoNamePrefix + "AngularMomentumMinimizationCostToGo", registry);
      }
      else
      {
         nominalEndingCornerPoint = null;
         nominalReferenceICP = null;
         nominalReferenceICPVelocity = null;
         nominalReferenceCMP = null;

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
      YoGraphicPosition referenceICP = new YoGraphicPosition(yoNamePrefix + "ReferenceICP", this.referenceICP, 0.005, YoAppearance.Yellow(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition referenceCMP = new YoGraphicPosition(yoNamePrefix + "ReferenceCMP", this.referenceCMP, 0.005, YoAppearance.Beige(), GraphicType.BALL_WITH_CROSS);

      artifactList.add(actualEndingCornerPoint.createArtifact());
      artifactList.add(referenceICP.createArtifact());
      artifactList.add(referenceCMP.createArtifact());
      yoGraphicsList.add(actualEndingCornerPoint);
      yoGraphicsList.add(referenceICP);
      yoGraphicsList.add(referenceCMP);

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

      yoGraphicsList.setVisible(visualize);
      artifactList.setVisible(visualize);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void updateCostsToGo(ICPQPOptimizationSolver solver)
   {
      if (debug)
      {
         residualCostToGo.set(solver.getCostToGo());
         costToGo.set(solver.getCostToGo());
         footstepCostToGo.set(solver.getFootstepCostToGo());
         feedbackCostToGo.set(solver.getFeedbackCostToGo());
         dynamicRelaxationCostToGo.set(solver.getDynamicRelaxationCostToGo());
         angularMomentumMinimizationCostToGo.set(solver.getAngularMomentumMinimizationCostToGo());
      }
   }

   public void extractFootstepSolutions(ArrayList<YoFramePoint2d> footstepSolutionsToPack, ArrayList<FramePoint2d> unclippedFootstepSolutionsToPack,
         ArrayList<YoFramePoint2d> referenceFootstepLocations, ArrayList<Footstep> upcomingFootsteps, int numberOfFootstepsToConsider,
         ICPQPOptimizationSolver solver)
   {
      boolean firstStepAdjusted = false;
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         solver.getFootstepSolutionLocation(i, locationSolution);

         RigidBodyTransform ankleToSole = transformsFromAnkleToSole.get(upcomingFootsteps.get(i).getRobotSide());
         upcomingFootsteps.get(i).getAnklePosition2d(upcomingFootstepLocation, ankleToSole);
         ReferenceFrame deadbandFrame = upcomingFootsteps.get(i).getSoleReferenceFrame();

         FramePoint2d referenceFootstepLocation = referenceFootstepLocations.get(i).getFrameTuple2d();
         clippedLocationSolution.set(locationSolution);
         boolean footstepWasAdjusted = applyLocationDeadband(clippedLocationSolution, upcomingFootstepLocation, referenceFootstepLocations.get(i).getFrameTuple2d(),
               deadbandFrame, footstepDeadband.getDoubleValue(), footstepSolutionResolution.getDoubleValue());

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

   public void computeReferenceValuesFromSolution(ArrayList<FramePoint2d> footstepSolutions, ICPOptimizationInputHandler inputHandler,
         StateMultiplierCalculator stateMultiplierCalculator, YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity,
         double omega0, int numberOfFootstepsToConsider)
   {
      ArrayList<FrameVector2d> entryOffsets = inputHandler.getEntryOffsets();
      ArrayList<FrameVector2d> exitOffsets = inputHandler.getExitOffsets();
      FramePoint2d stanceEntryCMP = inputHandler.getStanceEntryCMP();
      FramePoint2d stanceExitCMP = inputHandler.getStanceExitCMP();
      inputHandler.getFinalICP().getFrameTuple2d(tempPoint2d);

      stateMultiplierCalculator.reconstructICPCornerPoint(tmpEndPoint, tempPoint2d, footstepSolutions, entryOffsets, exitOffsets,
            numberOfFootstepsToConsider);
      stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tempVector2d, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
                  beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tempVector2d, omega0, tempPoint2d);

      actualEndingCornerPoint.set(tmpEndPoint);
      referenceICP.set(tmpReferencePoint);
      referenceICPVelocity.set(tempVector2d);
      referenceCMP.set(tempPoint2d);
   }

   public void computeNominalValues(ArrayList<YoFramePoint2d> upcomingFootstepLocations, ICPOptimizationInputHandler inputHandler,
         StateMultiplierCalculator stateMultiplierCalculator, YoFramePoint2d beginningOfStateICP, YoFrameVector2d beginningOfStateICPVelocity,
         double omega0, int numberOfFootstepsToConsider)
   {
      if (debug)
      {
         ArrayList<FrameVector2d> entryOffsets = inputHandler.getEntryOffsets();
         ArrayList<FrameVector2d> exitOffsets = inputHandler.getExitOffsets();
         FramePoint2d stanceEntryCMP = inputHandler.getStanceEntryCMP();
         FramePoint2d stanceExitCMP = inputHandler.getStanceExitCMP();
         inputHandler.getFinalICP().getFrameTuple2d(tempPoint2d);

         stateMultiplierCalculator
               .yoReconstructICPCornerPoint(tmpEndPoint, tempPoint2d, upcomingFootstepLocations, entryOffsets, exitOffsets,
                     numberOfFootstepsToConsider);
         stateMultiplierCalculator.reconstructReferenceICP(tmpReferencePoint, tempVector2d, tmpEndPoint, stanceEntryCMP, stanceExitCMP,
               beginningOfStateICP.getFrameTuple2d(), beginningOfStateICPVelocity.getFrameTuple2d());

         CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tempVector2d, omega0, tempPoint2d);

         nominalEndingCornerPoint.set(tmpEndPoint);
         nominalReferenceICP.set(tmpReferencePoint);
         nominalReferenceICPVelocity.set(tempVector2d);
         nominalReferenceCMP.set(tempPoint2d);
      }
   }

   public void setReferenceValues(FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, double omega0)
   {
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega0, tempPoint2d);

      referenceICP.set(desiredICP);
      referenceICPVelocity.set(desiredICPVelocity);
      referenceCMP.set(tempPoint2d);
      if (debug)
      {
         nominalReferenceICP.set(desiredICP);
         nominalReferenceICPVelocity.set(desiredICPVelocity);
         nominalReferenceCMP.set(tempPoint2d);
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

