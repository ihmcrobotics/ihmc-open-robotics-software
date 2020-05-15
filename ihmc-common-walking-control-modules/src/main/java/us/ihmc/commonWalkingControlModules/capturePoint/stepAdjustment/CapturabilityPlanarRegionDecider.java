package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.List;

public class CapturabilityPlanarRegionDecider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;

   private static final double minimumIntersectionForSearch = 0.01;

   private static final double distanceFromEdgeOfPolygonForStepping = 0.06;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();
   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   private final DoubleProvider maxAngleForSteppable;
   private final DoubleProvider minimumAreaForSteppable;
   private final BooleanProvider usePlanarRegionConstraints;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final RecyclingArrayList<PlanarRegion> allPlanarRegionsThatAreSteppable = new RecyclingArrayList<>(PlanarRegion.class);
   private final YoInteger numberOfPlanarListsToConsider;

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;

   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();

   private final YoBoolean constraintRegionChanged;

   private final ConvexPolygon2D convexHullConstraint = new ConvexPolygon2D();
   private final ConvexPolygon2D convexHullConstraintInControlPlane = new ConvexPolygon2D();

   private final YoFrameConvexPolygon2D yoConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoShrunkConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoConvexHullConstraintInControlPlane;
   private final YoFrameConvexPolygon2D yoShrunkConvexHullConstraintInControlPlane;

   private final ICPControlPlane icpControlPlane;

   private PlanarRegion planarRegionToConstrainTo = null;

   public CapturabilityPlanarRegionDecider(ICPOptimizationParameters optimizationParameters,
                                           OneStepCaptureRegionCalculator captureRegionCalculator,
                                           StepAdjustmentReachabilityConstraint reachabilityConstraint,
                                           ICPControlPlane icpControlPlane,
                                           SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                           String yoNamePrefix,
                                           YoVariableRegistry registry,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.captureRegionCalculator = captureRegionCalculator;
      this.reachabilityConstraint = reachabilityConstraint;
      this.icpControlPlane = icpControlPlane;
      this.contactableFeet = contactableFeet;

      maxAngleForSteppable = new DoubleParameter(yoNamePrefix + "MaxAngleForSteppable", registry, maxNormalAngleFromVertical);
      minimumAreaForSteppable = new DoubleParameter(yoNamePrefix + "MinimumAreaForSteppable", registry, minimumAreaToConsider);

      numberOfPlanarListsToConsider = new YoInteger(yoNamePrefix + "NumberOfPlanarListsToConsider", registry);
      constraintRegionChanged = new YoBoolean(yoNamePrefix + "ConstraintRegionChanged", registry);
      usePlanarRegionConstraints = new BooleanParameter(yoNamePrefix + "UsePlanarRegionConstraints",
                                                        registry,
                                                        optimizationParameters.allowUsePlanarRegionConstraints());

      switchPlanarRegionConstraintsAutomatically = new YoBoolean(yoNamePrefix + "SwitchPlanarRegionConstraintsAutomatically", registry);
      switchPlanarRegionConstraintsAutomatically.set(optimizationParameters.switchPlanarRegionConstraintsAutomatically());

      yoConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraint", "", worldFrame, 12, registry);
      yoShrunkConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ShrunkConvexHullConstraint", "", worldFrame, 12, registry);
      yoConvexHullConstraintInControlPlane = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraintInControlPlane", "", worldFrame, 12, registry);
      yoShrunkConvexHullConstraintInControlPlane = new YoFrameConvexPolygon2D(yoNamePrefix + "ShrunkConvexHullConstraintInControlPlane",
                                                                              "",
                                                                              worldFrame,
                                                                              12,
                                                                              registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         YoArtifactPolygon shrunkActivePlanarRegionViz = new YoArtifactPolygon("ShrunkConvexHullConstraint",
                                                                               yoShrunkConvexHullConstraint,
                                                                               Color.RED,
                                                                               false,
                                                                               true);

         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ConvexHullConstraintInControlPlane",
                                                                                       yoConvexHullConstraintInControlPlane,
                                                                                       Color.PINK,
                                                                                       false);
         YoArtifactPolygon shrunkActivePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ShrunkConvexHullConstraintInControlPlane",
                                                                                             yoShrunkConvexHullConstraintInControlPlane,
                                                                                             Color.PINK,
                                                                                             false,
                                                                                             true);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), shrunkActivePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), shrunkActivePlanarRegionInControlPlaneViz);
      }
   }

   private final Vector3D planeNormal = new Vector3D();
   private final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   public void setPlanarRegions(java.util.List<PlanarRegion> planarRegions)
   {
      allPlanarRegionsThatAreSteppable.clear();
      numberOfPlanarListsToConsider.set(0);

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);

         if (isRegionValidForStepping(planarRegion))
         {
            allPlanarRegionsThatAreSteppable.add().set(planarRegions.get(i));
            numberOfPlanarListsToConsider.increment();
         }
      }
   }

   public void reset()
   {
      planarRegionToConstrainTo = null;
      yoConvexHullConstraint.clear();
   }

   public FrameConvexPolygon2DReadOnly updatePlanarRegionConstraintForStep(RobotSide upcomingFootstepSide,
                                                                           FramePose3DReadOnly footstepPose,
                                                                           java.util.List<Point2D> predictedContactPoints)
   {
      constraintRegionChanged.set(false);

      if (!usePlanarRegionConstraints.getValue())
         return null;

      boolean planarRegionIsValid = false;

      computeValidCaptureRegion();

      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose);
         computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo, upcomingFootstepSide, predictedContactPoints, footstepPose.getOrientation());
      }

      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue())
      {
         if (planarRegionToConstrainTo != null)
            planarRegionIsValid = checkIfCurrentPlanarRegionIsValid();

         if (!planarRegionIsValid)
         {
            PlanarRegion betterRegion = findBestPlanarRegionToStepTo();
            if (betterRegion != null)
            {
               planarRegionToConstrainTo = betterRegion;
               computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo, upcomingFootstepSide, predictedContactPoints, footstepPose.getOrientation());
               constraintRegionChanged.set(true);
            }
         }
      }

      if (planarRegionToConstrainTo != null)
      {
         return yoShrunkConvexHullConstraintInControlPlane;
      }

      return null;
   }

   private final ConvexPolygon2D footstepPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform orientationTransform = new RigidBodyTransform();

   private void computeShrunkAndProjectedConvexHulls(PlanarRegion planarRegion,
                                                     RobotSide upcomingFootstepSide,
                                                     java.util.List<? extends Point2DBasics> predictedContactPoints,
                                                     Orientation3DReadOnly orientation)
   {
      computeFootstepPolygon(upcomingFootstepSide, predictedContactPoints, orientation);

      yoConvexHullConstraint.set(planarRegion.getConvexHull());
      yoConvexHullConstraint.applyTransform(planarRegion.getTransformToWorld(), false);

      icpControlPlane.projectConvexHullOntoControlPlane(planarRegion.getConvexHull(), planarRegion.getTransformToWorld(), yoConvexHullConstraintInControlPlane);

      scaler.scaleConvexPolygonToContainInteriorPolygon(yoConvexHullConstraint,
                                                        footstepPolygon,
                                                        distanceFromEdgeOfPolygonForStepping,
                                                        yoShrunkConvexHullConstraint);

      icpControlPlane.projectPlanarRegionConvexHullInWorldOntoControlPlane(yoShrunkConvexHullConstraint,
                                                                           planarRegion,
                                                                           yoShrunkConvexHullConstraintInControlPlane);
   }

   private void computeFootstepPolygon(RobotSide upcomingFootstepSide, List<? extends Point2DBasics> predictedContactPoints, Orientation3DReadOnly orientation)
   {
      if (predictedContactPoints.isEmpty())
         predictedContactPoints = contactableFeet.get(upcomingFootstepSide).getContactPoints2d();

      footstepPolygon.clear();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         footstepPolygon.addVertex(predictedContactPoints.get(i));
      footstepPolygon.update();

      orientationTransform.getRotation().set(orientation);

      footstepPolygon.applyTransform(orientationTransform);
   }

   private void computeValidCaptureRegion()
   {
      FrameConvexPolygon2D fullCaptureRegion = captureRegionCalculator.getCaptureRegion();
      fullCaptureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      reachabilityConstraint.getReachabilityConstraint();

      polygonTools.computeIntersectionOfPolygons(fullCaptureRegion, reachabilityConstraint.getReachabilityConstraint(), captureRegion);
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      planarRegion.getNormal(planeNormal);

      double angle = planeNormal.angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      // TODO switch to the concave hull
      return planarRegion.getConvexHull().getArea() > minimumAreaForSteppable.getValue();
   }

   private PlanarRegion findPlanarRegionUnderFoothold(FramePose3DReadOnly foothold)
   {
      PlanarRegion highestRegionUnderFoot = null;
      double highestPoint = Double.NEGATIVE_INFINITY;
      for (int regionIndex = 0; regionIndex < allPlanarRegionsThatAreSteppable.size(); regionIndex++)
      {
         PlanarRegion planarRegion = allPlanarRegionsThatAreSteppable.get(regionIndex);

         if (!planarRegion.isPointInWorld2DInside(foothold.getPosition()))
            continue;

         double height = planarRegion.getPlaneZGivenXY(foothold.getPosition().getX(), foothold.getPosition().getY());
         if (height >= highestPoint)
         {
            highestPoint = height;
            highestRegionUnderFoot = planarRegion;
         }
      }

      return highestRegionUnderFoot;
   }

   private boolean checkIfCurrentPlanarRegionIsValid()
   {
      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, yoConvexHullConstraintInControlPlane);

      return intersectionArea > minimumIntersectionForSearch;
   }

   private PlanarRegion findBestPlanarRegionToStepTo()
   {
      double maxArea = 0.0;
      PlanarRegion activePlanarRegion = null;

      for (int regionIndex = 0; regionIndex < allPlanarRegionsThatAreSteppable.size(); regionIndex++)
      {
         PlanarRegion planarRegion = allPlanarRegionsThatAreSteppable.get(regionIndex);
         convexHullConstraint.set(planarRegion.getConvexHull());
         convexHullConstraint.applyTransform(planarRegion.getTransformToWorld(), false);

         icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, convexHullConstraintInControlPlane);

         double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, convexHullConstraintInControlPlane);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = planarRegion;
         }
      }

      return activePlanarRegion;
   }
}
