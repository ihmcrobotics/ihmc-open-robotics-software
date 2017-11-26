package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

import java.awt.*;

public class PlanarRegionConstraintProvider
{
   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double distanceFromEdgeForStepping = 0.06;
   private static final double distanceFromEdgeForSwitching = 0.03;

   private static final double minimumAreaForSearch = 0.01;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameConvexPolygon2d yoActivePlanarRegion;
   private final YoFrameConvexPolygon2d yoActivePlanarRegionInControlPlane;
   private final YoFrameConvexPolygon2d yoShrunkActivePlanarRegion;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final YoDouble distanceToPlanarRegionEdgeForNoOverhang;
   private final YoInteger numberOfPlanarListsToConsider;

   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ICPControlPlane icpControlPlane;

   private PlanarRegion activePlanarRegion = null;

   private final FrameConvexPolygon2d activePlanarRegionConvexHull = new FrameConvexPolygon2d();

   private final ConvexPolygon2D activePlanarRegionConvexHullInControlFrame = new ConvexPolygon2D();
   private final ConvexPolygon2D projectedAndShrunkConvexHull = new ConvexPolygon2D();

   private final RigidBodyTransform planeTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame planeReferenceFrame;

   private final ConvexPolygonToolbox convexPolygonToolbox = new ConvexPolygonToolbox();

   public PlanarRegionConstraintProvider(ICPControlPlane icpControlPlane, WalkingControllerParameters parameters, BipedSupportPolygons bipedSupportPolygons,
                                         SideDependentList<? extends ContactablePlaneBody> contactableFeet, String yoNamePrefix, boolean visualize,
                                         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPlane;
      this.contactableFeet = contactableFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;

      captureRegionCalculator = new OneStepCaptureRegionCalculator(bipedSupportPolygons, parameters, yoNamePrefix, registry, yoGraphicsListRegistry);

      yoActivePlanarRegion = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraint", "", worldFrame, 12, registry);
      yoShrunkActivePlanarRegion = new YoFrameConvexPolygon2d(yoNamePrefix + "ShrunkActivePlanarRegionConstraint", "", worldFrame, 12, registry);
      yoActivePlanarRegionInControlPlane = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraintInControlPlane", "", worldFrame, 12, registry);

      distanceToPlanarRegionEdgeForNoOverhang = new YoDouble(yoNamePrefix + "DistanceToPlanarRegionEdgeForNoOverhang", registry);
      numberOfPlanarListsToConsider = new YoInteger(yoNamePrefix + "NumberOfPlanarListsToConsider", registry);

      planeReferenceFrame = new ReferenceFrame("planeReferenceFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(planeTransformToWorld);
         }
      };

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ActivePlanarRegionViz", yoActivePlanarRegion, Color.RED, false, true);
         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ActivePlanarRegionInControlPlaneViz", yoActivePlanarRegionInControlPlane, Color.RED, false);
         YoArtifactPolygon shrunkPlanarViz = new YoArtifactPolygon("ShrunkActivePlanarRegionInControlPlaneViz", yoShrunkActivePlanarRegion, Color.PINK, false);

         activePlanarRegionViz.setVisible(visualize);
         activePlanarRegionInControlPlaneViz.setVisible(visualize);
         shrunkPlanarViz.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), shrunkPlanarViz);
      }
   }

   // TODO change this functionality
   public void setActivePlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionsList.clear();

      if (planarRegion != null)
      {
         planarRegionsList.addPlanarRegion(planarRegion);
      }
      else
      {
         yoActivePlanarRegion.clearAndHide();
         yoShrunkActivePlanarRegion.clearAndHide();
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
      reset();

      captureRegionCalculator.hideCaptureRegion();
      solver.resetPlanarRegionConstraint();

      yoActivePlanarRegion.clearAndHide();
      yoActivePlanarRegionInControlPlane.clearAndHide();
      yoShrunkActivePlanarRegion.clearAndHide();
   }


   public void updatePlanarRegionConstraintForSingleSupport(Footstep footstep, double swingTimeRemaining, FramePoint2D currentICP, double omega0, SimpleICPOptimizationQPSolver solver)
   {
      captureRegionCalculator.calculateCaptureRegion(footstep.getRobotSide(), swingTimeRemaining, currentICP, omega0,
                                                     bipedSupportPolygons.getFootPolygonInWorldFrame(footstep.getRobotSide().getOppositeSide()));

      solver.resetPlanarRegionConstraint();

      boolean planarRegionNeedsUpdating = true;

      if (activePlanarRegion == null)
         findPlanarRegionAttachedToFootstep(footstep);

      if (activePlanarRegion != null)
         planarRegionNeedsUpdating = checkCurrentPlanarRegion();

      if (planarRegionNeedsUpdating)
         activePlanarRegion = findPlanarRegionWithLargestIntersectionArea();

      if (activePlanarRegion != null)
      {
         ConvexPolygon2D projectedAndShrunkPlanarRegion = computeShrunkAndProjectedConvexHull(activePlanarRegion, footstep);
         solver.setPlanarRegionConstraint(projectedAndShrunkPlanarRegion);
      }
   }


   // FIXME does not account for yawing of the foot
   private final ConvexPolygon2D footstepPolygon = new ConvexPolygon2D();
   private ConvexPolygon2D computeShrunkAndProjectedConvexHull(PlanarRegion planarRegion, Footstep footstep)
   {
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null)
      {
         footstepPolygon.clear();
         for (int i = 0; i < predictedContactPoints.size(); i++)
            footstepPolygon.addVertex(predictedContactPoints.get(i));
         footstepPolygon.update();
      }
      else
      {
         footstepPolygon.clear();
         List<FramePoint2D> contactPoints = contactableFeet.get(footstep.getRobotSide()).getContactPoints2d();
         //these are in the sole frame
         for (int i = 0; i < contactPoints.size(); i++)
         {
            FramePoint2D contactPoint = contactPoints.get(i);
            footstepPolygon.addVertex(contactPoint);
         }
         footstepPolygon.update();
      }
      icpControlPlane.scaleAndProjectPlanarRegionConvexHullOntoControlPlane(planarRegion, footstepPolygon, projectedAndShrunkConvexHull, distanceFromEdgeForStepping);
      yoShrunkActivePlanarRegion.setConvexPolygon2d(projectedAndShrunkConvexHull);

      return projectedAndShrunkConvexHull;
   }


   private final ConvexPolygon2D tempProjectedPolygon = new ConvexPolygon2D();

   private final FramePoint2D tempPoint = new FramePoint2D(worldFrame);


   public void findPlanarRegionAttachedToFootstep(Footstep footstep)
   {
      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         planarRegion.getTransformToWorld(planeTransformToWorld);
         planeReferenceFrame.update();

         footstep.getPosition2d(tempPoint);
         tempPoint.changeFrame(planeReferenceFrame);

         if (planarRegion.isPointInside(tempPoint.getPoint()))
         {
            activePlanarRegion = planarRegion;

            icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, tempProjectedPolygon);
            activePlanarRegionConvexHullInControlFrame.setAndUpdate(tempProjectedPolygon);
            break;
         }

      }
   }

   /**
    * Returns whether or not the current planar region needs updating
    */
   private boolean checkCurrentPlanarRegion()
   {
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      icpControlPlane.scaleAndProjectPlanarRegionConvexHullOntoControlPlane(activePlanarRegion, tempProjectedPolygon, distanceFromEdgeForSwitching);

      double intersectionArea = convexPolygonToolbox.computeIntersectionAreaOfPolygons(captureRegion.getConvexPolygon2d(), tempProjectedPolygon);

      if (intersectionArea > minimumAreaForSearch)
      {
         activePlanarRegionConvexHull.setIncludingFrame(planeReferenceFrame, activePlanarRegion.getConvexHull());
         activePlanarRegionConvexHull.changeFrameAndProjectToXYPlane(worldFrame);

         yoActivePlanarRegion.setConvexPolygon2d(activePlanarRegionConvexHull.getConvexPolygon2d());

         icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(activePlanarRegion, activePlanarRegionConvexHullInControlFrame);
         yoActivePlanarRegionInControlPlane.setConvexPolygon2d(activePlanarRegionConvexHullInControlFrame);
         return false;
      }

      return true;
   }

   /**
    * returns whether or not there is a planar region that intersects the capture region
    */
   private PlanarRegion findPlanarRegionWithLargestIntersectionArea()
   {
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      double maxArea = 0.0;
      PlanarRegion activePlanarRegion = null;
      activePlanarRegionConvexHullInControlFrame.clear();

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);

         icpControlPlane.scaleAndProjectPlanarRegionConvexHullOntoControlPlane(planarRegion, tempProjectedPolygon, distanceFromEdgeForSwitching);

         double intersectionArea = convexPolygonToolbox.computeIntersectionAreaOfPolygons(captureRegion.getConvexPolygon2d(), tempProjectedPolygon);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = planarRegion;

            icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(activePlanarRegion, tempProjectedPolygon);
            activePlanarRegionConvexHullInControlFrame.setAndUpdate(tempProjectedPolygon);
         }
      }

      if (activePlanarRegion == null)
         activePlanarRegion = this.activePlanarRegion;

      if (activePlanarRegion != null)
      {
         activePlanarRegion.getTransformToWorld(planeTransformToWorld);
         planeReferenceFrame.update();
         activePlanarRegionConvexHull.setIncludingFrame(planeReferenceFrame, activePlanarRegion.getConvexHull());
         activePlanarRegionConvexHull.changeFrameAndProjectToXYPlane(worldFrame);

         yoActivePlanarRegion.setConvexPolygon2d(activePlanarRegionConvexHull.getConvexPolygon2d());
         yoActivePlanarRegionInControlPlane.setConvexPolygon2d(activePlanarRegionConvexHullInControlFrame);
      }
      else
      {
         yoActivePlanarRegion.clearAndHide();
         yoShrunkActivePlanarRegion.clearAndHide();
         yoActivePlanarRegionInControlPlane.clearAndHide();
      }

      return activePlanarRegion;
   }

   public PlanarRegion getActivePlanarRegion()
   {
      return activePlanarRegion;
   }

   public void reset()
   {
      activePlanarRegion = null;
      activePlanarRegionConvexHullInControlFrame.clear();

      yoActivePlanarRegion.clearAndHide();
      yoShrunkActivePlanarRegion.clearAndHide();
      yoActivePlanarRegionInControlPlane.clearAndHide();
   }
}
