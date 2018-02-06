package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICPOptimizationSolutionHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ICPControlPlane icpControlPlane;

   private final YoBoolean useICPControlPolygons;

   private final YoDouble footstepDeadband;
   private final YoDouble footstepSolutionResolution;

   private final YoBoolean footstepWasAdjusted;
   private final YoFrameVector2d footstepAdjustment;
   private final YoFrameVector2d clippedFootstepAdjustment;

   private final YoDouble residualCostToGo;
   private final YoDouble costToGo;
   private final YoDouble footstepCostToGo;
   private final YoDouble copFeedbackCostToGo;
   private final YoDouble cmpFeedbackCostToGo;

   private final YoFramePoint2d adjustedICPReferenceLocation;
   private final YoFramePoint2d footstepSolutionInControlPlane;

   private final boolean debug;

   private final FramePoint2D locationSolutionOnPlane = new FramePoint2D();
   private final FramePoint3D locationSolution = new FramePoint3D();
   private final FramePoint2D previousLocationSolution = new FramePoint2D();
   private final FramePoint2D clippedLocationSolution = new FramePoint2D();

   private final FramePoint3D solutionLocation = new FramePoint3D();
   private final FramePoint3D referenceLocation = new FramePoint3D();
   private final FramePoint3D previousLocation = new FramePoint3D();
   private final FrameVector3D solutionAdjustment = new FrameVector3D();

   private final FrameVector3D tempVector = new FrameVector3D();

   private final String yoNamePrefix;

   public ICPOptimizationSolutionHandler(ICPOptimizationParameters icpOptimizationParameters, YoBoolean useICPControlPolygons, String yoNamePrefix,
                                         YoVariableRegistry registry)
   {
      this(null, icpOptimizationParameters, useICPControlPolygons, false, yoNamePrefix, registry);
   }

   public ICPOptimizationSolutionHandler(ICPControlPlane icpControlPlane, ICPOptimizationParameters icpOptimizationParameters, YoBoolean useICPControlPolygons,
                                         boolean debug, String yoNamePrefix, YoVariableRegistry registry)
   {
      this.useICPControlPolygons = useICPControlPolygons;
      this.yoNamePrefix = yoNamePrefix;
      this.debug = debug;
      this.icpControlPlane = icpControlPlane;

      if (debug)
      {
         residualCostToGo = new YoDouble(yoNamePrefix + "ResidualCostToGo", registry);
         costToGo = new YoDouble(yoNamePrefix + "CostToGo", registry);
         footstepCostToGo = new YoDouble(yoNamePrefix + "FootstepCostToGo", registry);
         copFeedbackCostToGo = new YoDouble(yoNamePrefix + "CoPFeedbackCostToGo", registry);
         cmpFeedbackCostToGo = new YoDouble(yoNamePrefix + "CMPFeedbackCostToGo", registry);
      }
      else
      {
         residualCostToGo = null;
         costToGo = null;
         footstepCostToGo = null;
         copFeedbackCostToGo = null;
         cmpFeedbackCostToGo = null;
      }

      footstepDeadband = new YoDouble(yoNamePrefix + "FootstepDeadband", registry);
      footstepSolutionResolution = new YoDouble(yoNamePrefix + "FootstepSolutionResolution", registry);

      footstepWasAdjusted = new YoBoolean(yoNamePrefix + "FootstepWasAdjusted", registry);
      footstepAdjustment = new YoFrameVector2d(yoNamePrefix + "FootstepAdjustment", worldFrame, registry);
      clippedFootstepAdjustment = new YoFrameVector2d(yoNamePrefix + "ClippedFootstepAdjustment", worldFrame, registry);

      adjustedICPReferenceLocation = new YoFramePoint2d(yoNamePrefix + "AdjustedICPReferenceLocation", worldFrame, registry);
      footstepSolutionInControlPlane = new YoFramePoint2d(yoNamePrefix + "FootstepSolutionReturned", worldFrame, registry);

      footstepDeadband.set(icpOptimizationParameters.getAdjustmentDeadband());
      footstepSolutionResolution.set(icpOptimizationParameters.getFootstepSolutionResolution());
   }

   public void setupVisualizers(ArtifactList artifactList)
   {
      YoGraphicPosition adjustedICP = new YoGraphicPosition(yoNamePrefix + "AdjustedICPReferencedLocation", adjustedICPReferenceLocation, 0.01, YoAppearance.LightYellow(),
                                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition footstepPositionInControlPlane = new YoGraphicPosition(yoNamePrefix + "FootstepSolutionInControlPlane", footstepSolutionInControlPlane, 0.005,
                                                                               YoAppearance.ForestGreen(), YoGraphicPosition.GraphicType.SOLID_BALL);

      artifactList.add(adjustedICP.createArtifact());
      artifactList.add(footstepPositionInControlPlane.createArtifact());
   }

   public void updateCostsToGo(ICPOptimizationQPSolver solver)
   {
      if (debug)
      {
         residualCostToGo.set(solver.getCostToGo());
         costToGo.set(solver.getCostToGo());
         footstepCostToGo.set(solver.getFootstepCostToGo());
         copFeedbackCostToGo.set(solver.getCoPFeedbackCostToGo());
         cmpFeedbackCostToGo.set(solver.getCMPFeedbackCostToGo());
      }
   }

   private final FramePoint3D referenceFootstepLocation = new FramePoint3D();
   private final FramePoint2D referenceFootstepLocation2D = new FramePoint2D();

   public void extractFootstepSolution(FixedFramePose3DBasics footstepSolutionToPack, FixedFrameTuple2DBasics unclippedFootstepSolutionToPack, Footstep upcomingFootstep,
                                       ICPOptimizationQPSolver solver)
   {
      upcomingFootstep.getPosition(referenceFootstepLocation);
      referenceFootstepLocation2D.set(referenceFootstepLocation);

      solver.getFootstepSolutionLocation(0, locationSolutionOnPlane);
      footstepSolutionInControlPlane.set(locationSolutionOnPlane);

      if (useICPControlPolygons.getBooleanValue())
         icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, locationSolutionOnPlane, locationSolution, referenceFootstepLocation.getZ());
      else
         locationSolution.set(locationSolutionOnPlane);

      ReferenceFrame deadbandFrame = upcomingFootstep.getSoleReferenceFrame();
      previousLocationSolution.set(footstepSolutionToPack.getPosition());
      clippedLocationSolution.set(locationSolution);
      boolean footstepWasAdjusted = applyLocationDeadband(clippedLocationSolution, previousLocationSolution, referenceFootstepLocation2D,
                                                          deadbandFrame, footstepDeadband.getDoubleValue(), footstepSolutionResolution.getDoubleValue());

      footstepAdjustment.set(locationSolution);
      footstepAdjustment.sub(referenceFootstepLocation2D);
      clippedFootstepAdjustment.sub(clippedLocationSolution, referenceFootstepLocation2D);

      footstepSolutionToPack.setPosition(clippedLocationSolution);
      unclippedFootstepSolutionToPack.set(locationSolution);

      this.footstepWasAdjusted.set(footstepWasAdjusted);
   }

   // fixme this is wrong
   public void extractFootstepSolution(FixedFramePose3DBasics footstepSolutionToPack, FixedFrameTuple2DBasics unclippedFootstepSolutionToPack, Footstep upcomingFootstep,
                                       PlanarRegion activePlanarRegion, ICPOptimizationQPSolver solver)
   {
      if (activePlanarRegion == null)
      {
         extractFootstepSolution(footstepSolutionToPack, unclippedFootstepSolutionToPack, upcomingFootstep, solver);
         return;
      }

      upcomingFootstep.getPosition(referenceFootstepLocation);
      referenceFootstepLocation2D.set(referenceFootstepLocation);

      solver.getFootstepSolutionLocation(0, locationSolutionOnPlane);
      footstepSolutionInControlPlane.set(locationSolutionOnPlane);

      if (useICPControlPolygons.getBooleanValue())
         icpControlPlane.projectPointFromPlaneOntoPlanarRegion(worldFrame, locationSolutionOnPlane, locationSolution, activePlanarRegion);
      else
         locationSolution.set(locationSolutionOnPlane);

      ReferenceFrame deadbandFrame = upcomingFootstep.getSoleReferenceFrame();
      previousLocationSolution.set(footstepSolutionToPack.getPosition());

      clippedLocationSolution.set(locationSolution);
      boolean footstepWasAdjusted = applyLocationDeadband(clippedLocationSolution, previousLocationSolution, referenceFootstepLocation2D,
                                                          deadbandFrame, footstepDeadband.getDoubleValue(), footstepSolutionResolution.getDoubleValue());

      footstepAdjustment.set(locationSolution);
      footstepAdjustment.sub(referenceFootstepLocation2D);
      clippedFootstepAdjustment.sub(clippedLocationSolution, referenceFootstepLocation2D);

      footstepSolutionToPack.setPosition(clippedLocationSolution);
      unclippedFootstepSolutionToPack.set(locationSolution);

      this.footstepWasAdjusted.set(footstepWasAdjusted);
   }

   public void zeroAdjustment()
   {
      footstepAdjustment.setToZero();
      clippedFootstepAdjustment.setToZero();
      footstepWasAdjusted.set(false);
   }

   public void updateVisualizers(FramePoint2DReadOnly desiredICP, double footstepMultiplier)
   {
      adjustedICPReferenceLocation.scaleAdd(footstepMultiplier, clippedFootstepAdjustment, desiredICP);
   }

   private boolean applyLocationDeadband(FramePoint2D solutionLocationToModify, FramePoint2DReadOnly currentSolutionLocation, FramePoint2DReadOnly referenceLocation2d,
         ReferenceFrame deadbandFrame, double deadband, double deadbandResolution)
   {
      solutionLocation.setIncludingFrame(solutionLocationToModify, 0.0);
      referenceLocation.setIncludingFrame(referenceLocation2d, 0.0);
      previousLocation.setIncludingFrame(currentSolutionLocation, 0.0);

      solutionLocation.changeFrame(worldFrame);
      referenceLocation.changeFrame(worldFrame);
      previousLocation.changeFrame(worldFrame);

      solutionAdjustment.setToZero(worldFrame);
      solutionAdjustment.sub(solutionLocation, referenceLocation);
      solutionAdjustment.changeFrame(deadbandFrame);

      boolean wasAdjusted = false;

      if (solutionAdjustment.length() < deadband)
      {
         referenceLocation.changeFrame(solutionLocationToModify.getReferenceFrame());
         solutionLocationToModify.set(referenceLocation);

         return false;
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
      tempVector.sub(solutionLocation, previousLocation);
      tempVector.changeFrame(deadbandFrame);

      if (tempVector.length() < deadbandResolution)
         solutionLocation.set(previousLocation);
      else
         wasAdjusted = true;

      solutionLocation.changeFrame(solutionLocationToModify.getReferenceFrame());
      solutionLocationToModify.set(solutionLocation);

      return wasAdjusted;
   }

   public boolean wasFootstepAdjusted()
   {
      return footstepWasAdjusted.getBooleanValue();
   }

   public void setFootstepWasAdjustedBySnapper(boolean footstepWasAdjusted)
   {
      if (footstepWasAdjusted)
         this.footstepWasAdjusted.set(true);
   }

   public FrameVector2DReadOnly getFootstepAdjustment()
   {
      return footstepAdjustment;
   }

   public FrameVector2DReadOnly getClippedFootstepAdjustment()
   {
      return clippedFootstepAdjustment;
   }
}

