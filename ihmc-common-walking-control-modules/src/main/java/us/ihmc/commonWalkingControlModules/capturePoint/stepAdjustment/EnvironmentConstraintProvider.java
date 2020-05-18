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

public class EnvironmentConstraintProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;

   private static final double distanceFromEdgeOfPolygonForStepping = 0.06;

   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final DoubleProvider maxAngleForSteppable;
   private final DoubleProvider minimumAreaForSteppable;

   private final YoFrameConvexPolygon2D yoConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoShrunkConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoConvexHullConstraintInControlPlane;
   private final YoFrameConvexPolygon2D yoShrunkConvexHullConstraintInControlPlane;

   private final ICPControlPlane icpControlPlane;

   private final RecyclingArrayList<PlanarRegion> allPlanarRegionsThatAreSteppable = new RecyclingArrayList<>(PlanarRegion.class);

   private PlanarRegion planarRegionToConstrainTo = null;

   public EnvironmentConstraintProvider(ICPControlPlane icpControlPlane,
                                        SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                        String yoNamePrefix,
                                        YoVariableRegistry registry,
                                        YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = icpControlPlane;
      this.contactableFeet = contactableFeet;

      maxAngleForSteppable = new DoubleParameter(yoNamePrefix + "MaxAngleForSteppable", registry, maxNormalAngleFromVertical);
      minimumAreaForSteppable = new DoubleParameter(yoNamePrefix + "MinimumAreaForSteppable", registry, minimumAreaToConsider);

      yoConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraint", "", worldFrame, 12, registry);
      yoShrunkConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ShrunkConvexHullConstraint", "", worldFrame, 12, registry);
      yoConvexHullConstraintInControlPlane = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraintInControlPlane", "", worldFrame, 12, registry);
      yoShrunkConvexHullConstraintInControlPlane = new YoFrameConvexPolygon2D(yoNamePrefix + "ShrunkConvexHullConstraintInControlPlane", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         YoArtifactPolygon shrunkActivePlanarRegionViz = new YoArtifactPolygon("ShrunkConvexHullConstraint", yoShrunkConvexHullConstraint, Color.RED, false, true);

         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ConvexHullConstraintInControlPlane",
                                                                                       yoConvexHullConstraintInControlPlane, Color.PINK, false);
         YoArtifactPolygon shrunkActivePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ShrunkConvexHullConstraintInControlPlane",
                                                                                             yoShrunkConvexHullConstraintInControlPlane, Color.PINK, false, true);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), shrunkActivePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), shrunkActivePlanarRegionInControlPlaneViz);
      }
   }

   public void setPlanarRegionConstraint(PlanarRegion planarRegionToConstrainTo)
   {
      this.planarRegionToConstrainTo = planarRegionToConstrainTo;
   }

   public void setPlanarRegions(java.util.List<PlanarRegion> planarRegions)
   {
      allPlanarRegionsThatAreSteppable.clear();

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);

         if (isRegionValidForStepping(planarRegion))
         {
            allPlanarRegionsThatAreSteppable.add().set(planarRegions.get(i));
         }
      }
   }

   private final Vector3D planeNormal = new Vector3D();
   private final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      planarRegion.getNormal(planeNormal);

      double angle = planeNormal.angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      return planarRegion.getConvexHull().getArea() > minimumAreaForSteppable.getValue();
   }


   public void reset()
   {
      planarRegionToConstrainTo = null;
      yoConvexHullConstraint.clear();
      yoConvexHullConstraintInControlPlane.clear();
      yoShrunkConvexHullConstraint.clear();
      yoShrunkConvexHullConstraintInControlPlane.clear();
   }

   public FrameConvexPolygon2DReadOnly updatePlanarRegionConstraintForStep(RobotSide upcomingFootstepSide, FramePose3DReadOnly footstepPose,
                                                                           List<Point2D> predictedContactPoints)
   {
      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose);
      }

      if (planarRegionToConstrainTo != null)
      {
         computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo, upcomingFootstepSide, predictedContactPoints, footstepPose.getOrientation());
         return yoShrunkConvexHullConstraintInControlPlane;
      }

      return null;
   }

   private final ConvexPolygon2D footstepPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform orientationTransform = new RigidBodyTransform();

   private void computeShrunkAndProjectedConvexHulls(PlanarRegion planarRegion, RobotSide upcomingFootstepSide, List<? extends Point2DBasics> predictedContactPoints, Orientation3DReadOnly orientation)
   {
      computeFootstepPolygon(upcomingFootstepSide, predictedContactPoints, orientation);

      yoConvexHullConstraint.set(planarRegion.getConvexHull());
      yoConvexHullConstraint.applyTransform(planarRegion.getTransformToWorld(), false);

      icpControlPlane.projectConvexHullOntoControlPlane(planarRegion.getConvexHull(), planarRegion.getTransformToWorld(), yoConvexHullConstraintInControlPlane);

      scaler.scaleConvexPolygonToContainInteriorPolygon(yoConvexHullConstraint, footstepPolygon, distanceFromEdgeOfPolygonForStepping, yoShrunkConvexHullConstraint);

      icpControlPlane.projectPlanarRegionConvexHullInWorldOntoControlPlane(yoShrunkConvexHullConstraint, planarRegion, yoShrunkConvexHullConstraintInControlPlane);
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

      footstepPolygon.applyTransform(orientationTransform, false);
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
}
