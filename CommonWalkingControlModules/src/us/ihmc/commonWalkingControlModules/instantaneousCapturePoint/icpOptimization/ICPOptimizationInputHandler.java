package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ReferenceCentroidalMomentumPivotLocationsCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.StateMultiplierCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class ICPOptimizationInputHandler
{
   private static final String namePrefix = "icpOptimizationController";
   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint finalICP;
   private final YoFramePoint2d stanceEntryCMP;
   private final YoFramePoint2d stanceExitCMP;

   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;
   private final StateMultiplierCalculator stateMultiplierCalculator;

   private final DoubleYoVariable doubleSupportDuration;
   private final DoubleYoVariable singleSupportDuration;

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final ArrayList<YoFramePointInMultipleFrames> entryCornerPoints = new ArrayList<>();
   private final ArrayList<YoFramePointInMultipleFrames> exitCornerPoints = new ArrayList<>();

   private final ArrayList<FrameVector2d> entryOffsets = new ArrayList<>();
   private final ArrayList<FrameVector2d> exitOffsets = new ArrayList<>();

   public ICPOptimizationInputHandler(CapturePointPlannerParameters icpPlannerParameters, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, int maximumNumberOfFootstepsToConsider, StateMultiplierCalculator stateMultiplierCalculator,
         DoubleYoVariable doubleSupportDuration, DoubleYoVariable singleSupportDuration, DoubleYoVariable exitCMPDurationInPercentOfStepTime, boolean visualize,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.stateMultiplierCalculator = stateMultiplierCalculator;

      this.doubleSupportDuration = doubleSupportDuration;
      this.singleSupportDuration = singleSupportDuration;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());

      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            maximumNumberOfFootstepsToConsider, registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(),
            bipedSupportPolygons.getSoleZUpFrames().get(RobotSide.LEFT), bipedSupportPolygons.getSoleZUpFrames().get(RobotSide.RIGHT)};
      for (int i = 0; i < maximumNumberOfFootstepsToConsider - 1; i++)
      {
         YoFramePointInMultipleFrames earlyCornerPoint = new YoFramePointInMultipleFrames(yoNamePrefix + "EntryCornerPoints" + i, registry, framesToRegister);
         entryCornerPoints.add(earlyCornerPoint);

         YoFramePointInMultipleFrames lateCornerPoint = new YoFramePointInMultipleFrames(yoNamePrefix + "ExitCornerPoints" + i, registry, framesToRegister);
         exitCornerPoints.add(lateCornerPoint);
      }

      finalICP = new YoFramePoint(yoNamePrefix + "FinalICP", worldFrame, registry);
      stanceEntryCMP = new YoFramePoint2d(yoNamePrefix + "StanceEntryCMP", worldFrame, registry);
      stanceExitCMP = new YoFramePoint2d(yoNamePrefix + "StanceExitCMP", worldFrame, registry);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         entryOffsets.add(new FrameVector2d(worldFrame));
         exitOffsets.add(new FrameVector2d(worldFrame));
      }

      if (visualize && yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry, visualize);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition entryCMP = new YoGraphicPosition("entryCMP", stanceEntryCMP, 0.01, YoAppearance.Red(), GraphicType.SQUARE);
      YoGraphicPosition exitCMP = new YoGraphicPosition("exitCMP", stanceExitCMP, 0.01, YoAppearance.Red(), GraphicType.SQUARE);
      YoGraphicPosition finalICP = new YoGraphicPosition("finalICP", this.finalICP, 0.005, YoAppearance.Black(), GraphicType.SOLID_BALL);

      yoGraphicsList.add(finalICP);

      artifactList.add(entryCMP.createArtifact());
      artifactList.add(exitCMP.createArtifact());
      artifactList.add(finalICP.createArtifact());

      artifactList.setVisible(visualize);
      yoGraphicsList.setVisible(visualize);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearPlan()
   {
      referenceCMPsCalculator.clear();
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      referenceCMPsCalculator.addUpcomingFootstep(footstep);
   }

   public void initializeForDoubleSupport(boolean isStanding, boolean useTwoCMPs, RobotSide transferToSide, double omega0)
   {
      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoCMPs);
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding, transferToSide);
      referenceCMPsCalculator.update();

      updateCornerPoints(useTwoCMPs, omega0);
   }

   public void initializeForSingleSupport(boolean useTwoCMPs, RobotSide supportSide, double omega0)
   {
      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoCMPs);
      referenceCMPsCalculator.computeReferenceCMPsStartingFromSingleSupport(supportSide);
      referenceCMPsCalculator.update();

      updateCornerPoints(useTwoCMPs, omega0);
   }

   private void updateCornerPoints(boolean useTwoCMPs, double omega0)
   {
      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      if (useTwoCMPs)
      {
         CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, exitCornerPoints, referenceCMPsCalculator.getEntryCMPs(), referenceCMPsCalculator.getExitCMPs(),
               steppingDuration, exitCMPDurationInPercentOfStepTime.getDoubleValue(), omega0);
      }
      else
      {
         CapturePointTools.computeDesiredCornerPoints(entryCornerPoints, referenceCMPsCalculator.getEntryCMPs(), false, steppingDuration, omega0);
      }
   }

   public void update(double timeInCurrentState, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      referenceCMPsCalculator.update();
      updateCornerPoints(useTwoCMPs, omega0);

      stateMultiplierCalculator.computeCurrentMultipliers(timeInCurrentState, useTwoCMPs, isInTransfer, omega0);
   }

   public void computeFinalICPRecursion(FramePoint2d finalICPRecursionToPack, int numberOfFootstepsToConsider, boolean useTwoCMPs, boolean isInTransfer, double omega0)
   {
      computeFinalICP(numberOfFootstepsToConsider, useTwoCMPs, isInTransfer);

      finalICPRecursionToPack.setByProjectionOntoXYPlane(finalICP.getFrameTuple());

      finalICPRecursionToPack.scale(stateMultiplierCalculator.getFinalICPRecursionMultiplier());
      finalICPRecursionToPack.scale(stateMultiplierCalculator.getStateEndCurrentMultiplier());
   }

   private void computeFinalICP(int numberOfFootstepsToConsider, boolean useTwoCMPs, boolean isInTransfer)
   {
      if (!useTwoCMPs || isInTransfer)
         finalICP.set(entryCornerPoints.get(numberOfFootstepsToConsider + 1));
      else
         finalICP.set(exitCornerPoints.get(numberOfFootstepsToConsider + 1));
   }

   private final FramePoint2d stanceCMPProjection = new FramePoint2d();
   private final FramePoint2d beginningOfStateICPProjection = new FramePoint2d();
   private final FrameVector2d beginningOfStateICPVelocityProjection = new FrameVector2d();
   private final FramePoint2d cmpOffsetRecursionEffect = new FramePoint2d();

   public void computeCMPConstantEffects(FramePoint2d cmpConstantEffectsToPack, FramePoint2d beginningOfStateICP, FrameVector2d beginningOfStateICPVelocity,
         ArrayList<YoFramePoint2d> upcomingFootstepLocations, int numberOfFootstepsToConsider, boolean useTwoCMPs, boolean isInTransfer)
   {
      computeStanceCMPProjection(stanceCMPProjection, useTwoCMPs, isInTransfer);
      computeBeginningOfStateICPProjection(beginningOfStateICPProjection, beginningOfStateICP);
      computeBeginningOfStateICPVelocityProjection(beginningOfStateICPVelocityProjection, beginningOfStateICPVelocity);

      cmpConstantEffectsToPack.setToZero();
      cmpConstantEffectsToPack.add(stanceCMPProjection);
      cmpConstantEffectsToPack.add(beginningOfStateICPProjection);
      cmpConstantEffectsToPack.add(beginningOfStateICPVelocityProjection);

      if (useTwoCMPs)
      {
         computeCMPOffsetRecursionEffect(cmpOffsetRecursionEffect, upcomingFootstepLocations, numberOfFootstepsToConsider);
         cmpConstantEffectsToPack.add(cmpOffsetRecursionEffect);
      }
   }


   private final FramePoint2d stanceEntryCMP2d = new FramePoint2d(worldFrame);
   private final FramePoint2d stanceExitCMP2d = new FramePoint2d(worldFrame);

   private void computeStanceCMPProjection(FramePoint2d stanceCMPProjectionToPack, boolean useTwoCMPs, boolean isInTransfer)
   {
      if (useTwoCMPs)
      {
         if (isInTransfer)
         {
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(1).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(1).getFrameTuple();

            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

            this.stanceEntryCMP.set(stanceEntryCMP2d);
            this.stanceExitCMP.set(stanceExitCMP2d);
         }
         else
         {
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();

            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

            this.stanceEntryCMP.set(stanceEntryCMP2d);
            this.stanceExitCMP.set(stanceExitCMP2d);
         }
      }
      else
      {
         if (isInTransfer)
         {
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(1).getFrameTuple();

            stanceExitCMP2d.setToZero();
            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);

            this.stanceEntryCMP.set(stanceEntryCMP2d);
            this.stanceExitCMP.setToNaN();
         }
         else
         {
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();

            stanceExitCMP2d.setToZero();
            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);

            this.stanceExitCMP.setToNaN();
            this.stanceEntryCMP.set(stanceEntryCMP2d);
         }
      }

      double entryMultiplier, exitMultiplier;
      entryMultiplier = stateMultiplierCalculator.getEntryCMPCurrentMultiplier();
      exitMultiplier = stateMultiplierCalculator.getExitCMPCurrentMultiplier();

      double currentStateProjectionMultiplier = stateMultiplierCalculator.getStateEndCurrentMultiplier();

      entryMultiplier += currentStateProjectionMultiplier * stateMultiplierCalculator.getStanceEntryCMPRecursionMultiplier();
      exitMultiplier += currentStateProjectionMultiplier * stateMultiplierCalculator.getStanceExitCMPRecursionMultiplier();

      stanceEntryCMP2d.scale(entryMultiplier);
      stanceExitCMP2d.scale(exitMultiplier);

      stanceCMPProjectionToPack.setToZero();
      stanceCMPProjectionToPack.add(stanceEntryCMP2d);
      stanceCMPProjectionToPack.add(stanceExitCMP2d);
   }

   private void computeBeginningOfStateICPProjection(FramePoint2d beginningOfStateICPProjectionToPack, FramePoint2d beginningOfStateICP)
   {
      beginningOfStateICPProjectionToPack.set(beginningOfStateICP);
      beginningOfStateICPProjectionToPack.scale(stateMultiplierCalculator.getInitialICPCurrentMultiplier());
   }

   private void computeBeginningOfStateICPVelocityProjection(FrameVector2d beginningOfStateICPVelocityProjectionToPack, FrameVector2d beginningOfStateICPVelocity)
   {
      beginningOfStateICPVelocityProjectionToPack.set(beginningOfStateICPVelocity);
      beginningOfStateICPVelocityProjectionToPack.scale(stateMultiplierCalculator.getInitialICPVelocityCurrentMultiplier());
   }

   private final FramePoint2d totalOffsetEffect = new FramePoint2d();
   private void computeCMPOffsetRecursionEffect(FramePoint2d cmpOffsetRecursionEffectToPack, ArrayList<YoFramePoint2d> upcomingFootstepLocations,
         int numberOfFootstepsToConsider)
   {
      computeTwoCMPOffsets(upcomingFootstepLocations, numberOfFootstepsToConsider);

      cmpOffsetRecursionEffectToPack.setToZero();

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         totalOffsetEffect.set(exitOffsets.get(i));
         totalOffsetEffect.scale(stateMultiplierCalculator.getExitCMPRecursionMultiplier(i));

         cmpOffsetRecursionEffectToPack.add(totalOffsetEffect);

         totalOffsetEffect.set(entryOffsets.get(i));
         totalOffsetEffect.scale(stateMultiplierCalculator.getEntryCMPRecursionMultiplier(i));

         cmpOffsetRecursionEffectToPack.add(totalOffsetEffect);
      }
      cmpOffsetRecursionEffectToPack.scale(stateMultiplierCalculator.getStateEndCurrentMultiplier());
   }

   private void computeTwoCMPOffsets(ArrayList<YoFramePoint2d> upcomingFootstepLocations, int numberOfFootstepsToConsider)
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         FrameVector2d entryOffset = entryOffsets.get(i);
         FrameVector2d exitOffset = exitOffsets.get(i);

         entryOffset.setToZero(worldFrame);
         exitOffset.setToZero(worldFrame);

         entryOffset.setByProjectionOntoXYPlane(referenceCMPsCalculator.getEntryCMPs().get(i + 1).getFrameTuple());
         exitOffset.setByProjectionOntoXYPlane(referenceCMPsCalculator.getExitCMPs().get(i + 1).getFrameTuple());

         entryOffset.sub(upcomingFootstepLocations.get(i).getFrameTuple2d());
         exitOffset.sub(upcomingFootstepLocations.get(i).getFrameTuple2d());
      }
   }

   public ArrayList<FrameVector2d> getEntryOffsets()
   {
      return entryOffsets;
   }

   public ArrayList<FrameVector2d> getExitOffsets()
   {
      return exitOffsets;
   }

   public FramePoint2d getStanceEntryCMP()
   {
      return stanceEntryCMP.getFrameTuple2d();
   }

   public FramePoint2d getStanceExitCMP()
   {
      return stanceExitCMP.getFrameTuple2d();
   }

   public FramePoint getFinalICP()
   {
      return finalICP.getFrameTuple();
   }
}
