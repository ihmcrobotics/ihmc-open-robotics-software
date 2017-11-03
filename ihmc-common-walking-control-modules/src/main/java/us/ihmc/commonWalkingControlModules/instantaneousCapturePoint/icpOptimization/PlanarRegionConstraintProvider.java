package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionVisualizer;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
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

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final YoDouble distanceToPlanarRegionEdgeForNoOverhang;

   private final OneStepCaptureRegionCalculator captureRegionCalculator;

   public PlanarRegionConstraintProvider(WalkingControllerParameters parameters, BipedSupportPolygons bipedSupportPolygons,
                                         SideDependentList<? extends ContactablePlaneBody> contactableFeet, String yoNamePrefix, boolean visualize, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactableFeet = contactableFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;

      captureRegionCalculator = new OneStepCaptureRegionCalculator(bipedSupportPolygons, parameters, yoNamePrefix, registry, yoGraphicsListRegistry);

      activePlanarRegion = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraint", "", worldFrame, 12, registry);
      distanceToPlanarRegionEdgeForNoOverhang = new YoDouble(yoNamePrefix + "DistanceToPlanarRegionEdgeForNoOverhang", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ActivePlanarRegionViz", activePlanarRegion, Color.RED, false);

         reachabilityGraphic.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
      }
   }

   public void setActivePlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionsList.clear();

      if (planarRegion != null)
      {
         planarRegionsList.addPlanarRegion(planarRegion);
         activePlanarRegion.setConvexPolygon2d(planarRegion.getConvexHull());
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

   public void updatePlanarRegionConstraintForSingleSupport(RobotSide supportSide, double swingTimeRemaining, FramePoint2D currentICP, double omega0, SimpleICPOptimizationQPSolver solver)
   {
      captureRegionCalculator.calculateCaptureRegion(supportSide.getOppositeSide(), swingTimeRemaining, currentICP, omega0,
                                                     bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide));

      solver.resetPlanarRegionConstraint();
      if (!planarRegionsList.isEmpty())
      {
         solver.setPlanarRegionConstraint(planarRegionsList.getLastPlanarRegion(), distanceToPlanarRegionEdgeForNoOverhang.getDoubleValue());
         activePlanarRegion.setConvexPolygon2d(planarRegionsList.getLastPlanarRegion().getConvexHull());
      }
   }

   public void reset()
   {
      activePlanarRegion.clearAndHide();
   }
}
