package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.ojalgo.function.multiary.MultiaryFunction;
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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

import java.awt.*;

public class PlanarRegionConstraintProvider
{
   private static final double maxNormalAngleFromVertical = 0.3;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameConvexPolygon2d yoActivePlanarRegion;
   private final YoFrameConvexPolygon2d yoActivePlanarRegionInControlPlane;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final YoDouble distanceToPlanarRegionEdgeForNoOverhang;
   private final YoInteger numberOfPlanarListsToConsider;

   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ICPControlPlane icpControlPlane;

   private PlanarRegion activePlanarRegion = null;
   private final ConvexPolygon2D activePlanarRegionInControlFrame = new ConvexPolygon2D();

   private final ConvexPolygon2D projectedAndShrunkConvexHull = new ConvexPolygon2D();
   private final YoDouble footstepDeadband;

   public PlanarRegionConstraintProvider(ICPControlPlane icpControlPlane, WalkingControllerParameters parameters, BipedSupportPolygons bipedSupportPolygons,
                                         SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoDouble footstepDeadband, String yoNamePrefix, boolean visualize, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPlane;
      this.contactableFeet = contactableFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.footstepDeadband = footstepDeadband;

      captureRegionCalculator = new OneStepCaptureRegionCalculator(bipedSupportPolygons, parameters, yoNamePrefix, registry, yoGraphicsListRegistry);

      yoActivePlanarRegion = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraint", "", worldFrame, 12, registry);
      yoActivePlanarRegionInControlPlane = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraintInControlPlane", "", worldFrame, 12, registry);
      distanceToPlanarRegionEdgeForNoOverhang = new YoDouble(yoNamePrefix + "DistanceToPlanarRegionEdgeForNoOverhang", registry);
      numberOfPlanarListsToConsider = new YoInteger(yoNamePrefix + "NumberOfPlanarListsToConsider", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ActivePlanarRegionViz", yoActivePlanarRegion, Color.RED, false, true);
         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ActivePlanarRegionInControlPlaneViz", yoActivePlanarRegionInControlPlane, Color.RED, false);

         activePlanarRegionViz.setVisible(visualize);
         activePlanarRegionInControlPlaneViz.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
      }
   }

   // TODO change this functionality
   public void setActivePlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionsList.clear();

      if (planarRegion != null)
      {
         planarRegionsList.addPlanarRegion(planarRegion);
         computeProjectedAndShrunkConvexHull();
      }
      else
      {
         yoActivePlanarRegion.clearAndHide();
         yoActivePlanarRegionInControlPlane.clearAndHide();
      }
   }

   private final Vector3D planeNormal = new Vector3D();
   private final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);
   public void setPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {
      planarRegionsList.clear();
      numberOfPlanarListsToConsider.set(0);

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);
         planarRegion.getNormal(planeNormal);

         double angle = planeNormal.angle(verticalAxis);

         if (angle < maxNormalAngleFromVertical)
         {
            planarRegionsList.addPlanarRegion(planarRegions.get(i));
            numberOfPlanarListsToConsider.increment();
         }
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

      yoActivePlanarRegion.clearAndHide();
      yoActivePlanarRegionInControlPlane.clearAndHide();
   }

   public void updatePlanarRegionConstraintForSingleSupport(RobotSide supportSide, double swingTimeRemaining, FramePoint2D currentICP, double omega0, SimpleICPOptimizationQPSolver solver)
   {
      captureRegionCalculator.calculateCaptureRegion(supportSide.getOppositeSide(), swingTimeRemaining, currentICP, omega0,
                                                     bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide));

      solver.resetPlanarRegionConstraint();

      if (computeActivePlanarRegion())
      {
         ConvexPolygon2D projectedAndShrunkPlanarRegion = computeProjectedAndShrunkConvexHull();
         solver.setPlanarRegionConstraint(projectedAndShrunkPlanarRegion);
      }
   }

   private ConvexPolygon2D computeProjectedAndShrunkConvexHull()
   {
      icpControlPlane.scaleAndProjectPlanarRegionConvexHullOntoControlPlane(activePlanarRegion, projectedAndShrunkConvexHull, distanceToPlanarRegionEdgeForNoOverhang.getDoubleValue());

      return projectedAndShrunkConvexHull;
   }


   private final ConvexPolygon2D tempProjectedPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D tempIntersection = new ConvexPolygon2D();

   // FIXME this makes garbage
   /**
    * returns whether or not there is a planar region constraint
    * @return
    */
   private boolean computeActivePlanarRegion()
   {
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      boolean hasPlanarRegion = false;
      double maxArea = 0.0;
      activePlanarRegion = null;
      activePlanarRegionInControlFrame.clear();

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);
         icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, tempProjectedPolygon);

         // FIXME this makes a lot of garbage
         ConvexPolygonTools.computeIntersectionOfPolygons(captureRegion.getConvexPolygon2d(), tempProjectedPolygon, tempIntersection);
         double intersectionArea = tempIntersection.getArea();

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = planarRegion;
            activePlanarRegionInControlFrame.setAndUpdate(tempProjectedPolygon);
            hasPlanarRegion = true;
         }
      }

      if (activePlanarRegion != null)
      {
         this.yoActivePlanarRegion.setConvexPolygon2d(activePlanarRegion.getConvexHull());
         this.yoActivePlanarRegionInControlPlane.setConvexPolygon2d(activePlanarRegionInControlFrame);
      }
      else
      {
         this.yoActivePlanarRegion.clearAndHide();
         this.yoActivePlanarRegionInControlPlane.clearAndHide();
      }

      return hasPlanarRegion;
   }

   public PlanarRegion getActivePlanarRegion()
   {
      return activePlanarRegion;
   }

   public void reset()
   {
      planarRegionsList.clear();

      activePlanarRegion = null;
      activePlanarRegionInControlFrame.clear();

      yoActivePlanarRegion.clearAndHide();
      yoActivePlanarRegionInControlPlane.clearAndHide();
   }
}
