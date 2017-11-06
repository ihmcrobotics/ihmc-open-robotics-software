package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionVisualizer;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

import java.awt.*;

public class PlanarRegionConstraintProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameConvexPolygon2d activePlanarRegion;
   private final YoFrameConvexPolygon2d activePlanarRegionInControlPlane;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final YoDouble distanceToPlanarRegionEdgeForNoOverhang;

   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ICPControlPlane icpControlPlane;

   private final ConvexPolygon2D projectedConvexHull = new ConvexPolygon2D();
   private final YoDouble footstepDeadband;

   public PlanarRegionConstraintProvider(ICPControlPlane icpControlPlane, WalkingControllerParameters parameters, BipedSupportPolygons bipedSupportPolygons,
                                         SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoDouble footstepDeadband, String yoNamePrefix, boolean visualize, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPlane;
      this.contactableFeet = contactableFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.footstepDeadband = footstepDeadband;

      captureRegionCalculator = new OneStepCaptureRegionCalculator(bipedSupportPolygons, parameters, yoNamePrefix, registry, yoGraphicsListRegistry);

      activePlanarRegion = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraint", "", worldFrame, 12, registry);
      activePlanarRegionInControlPlane = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraintInControlPlane", "", worldFrame, 12, registry);
      distanceToPlanarRegionEdgeForNoOverhang = new YoDouble(yoNamePrefix + "DistanceToPlanarRegionEdgeForNoOverhang", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ActivePlanarRegionViz", activePlanarRegion, Color.RED, false, true);
         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ActivePlanarRegionInControlPlaneViz", activePlanarRegionInControlPlane, Color.RED, false);

         activePlanarRegionViz.setVisible(visualize);
         activePlanarRegionInControlPlaneViz.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
      }
   }

   public void setActivePlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionsList.clear();

      if (planarRegion != null)
      {
         planarRegionsList.addPlanarRegion(planarRegion);

         activePlanarRegion.setConvexPolygon2d(planarRegion.getConvexHull());
         icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, projectedConvexHull);
         activePlanarRegionInControlPlane.setConvexPolygon2d(projectedConvexHull);
      }
      else
      {
         activePlanarRegion.clearAndHide();
      }
   }

   public void computeDistanceFromEdgeForNoOverhang(Footstep upcomingFootstep)
   {
      List<Point2D> predictedContactPoints = upcomingFootstep.getPredictedContactPoints();
      double maxDistance = 0.0;
      if (predictedContactPoints != null)
      {
         for (int i = 0; i < predictedContactPoints.size(); i++)
         {
            maxDistance = Math.max(maxDistance, predictedContactPoints.get(i).distanceFromOrigin());
         }
      }
      else
      {
         List<FramePoint2D> contactPoints = contactableFeet.get(upcomingFootstep.getRobotSide()).getContactPoints2d();
         for (int i = 0; i < contactPoints.size(); i++)
         {
            maxDistance = Math.max(maxDistance, contactPoints.get(i).distanceFromOrigin());
         }
      }
      distanceToPlanarRegionEdgeForNoOverhang.set(maxDistance);
   }


   public void updatePlanarRegionConstraintForDoubleSupport(SimpleICPOptimizationQPSolver solver)
   {
      captureRegionCalculator.hideCaptureRegion();
      solver.resetPlanarRegionConstraint();
   }

   // FIXME this is wrong
   public void updatePlanarRegionConstraintForSingleSupport(RobotSide supportSide, double swingTimeRemaining, FramePoint2D currentICP, double omega0, SimpleICPOptimizationQPSolver solver)
   {
      captureRegionCalculator.calculateCaptureRegion(supportSide.getOppositeSide(), swingTimeRemaining, currentICP, omega0,
                                                     bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide));

      solver.resetPlanarRegionConstraint();

      if (!planarRegionsList.isEmpty())
      {
         PlanarRegion planarRegion = planarRegionsList.getLastPlanarRegion();

         activePlanarRegion.setConvexPolygon2d(planarRegion.getConvexHull());
         icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, projectedConvexHull);

         activePlanarRegionInControlPlane.setConvexPolygon2d(projectedConvexHull);

         solver.setPlanarRegionConstraint(projectedConvexHull, distanceToPlanarRegionEdgeForNoOverhang.getDoubleValue() - footstepDeadband.getDoubleValue());
         activePlanarRegion.setConvexPolygon2d(planarRegionsList.getLastPlanarRegion().getConvexHull());
      }
   }

   public PlanarRegion getActivePlanarRegion()
   {
      return planarRegionsList.getLastPlanarRegion();
   }

   public void reset()
   {
      activePlanarRegion.clearAndHide();
   }
}
