package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

import java.awt.*;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.ToDoubleFunction;
import java.util.stream.Collectors;

public class CapturabilityBasedPlanarRegionDecider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;

   private static final double minimumIntersectionForSearch = 0.005;

   private static final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   private final YoDouble maxAngleForSteppable;
   private final YoDouble minimumAreaForSteppable;

   private List<PlanarRegion> steppableRegions = new ArrayList<>();

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean constraintRegionChanged;

   private final ConvexPolygon2D convexHullConstraintInControlPlane = new ConvexPolygon2D();

   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoConvexHullConstraintInControlPlane;

   private final ICPControlPlane icpControlPlane;

   private PlanarRegion planarRegionToConstrainTo = null;

   public CapturabilityBasedPlanarRegionDecider(ReferenceFrame centerOfMassFrame,
                                                double gravityZ,
                                                YoVariableRegistry registry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);

      maxAngleForSteppable = new YoDouble("maxAngleForSteppable", registry);
      minimumAreaForSteppable = new YoDouble("minimumAreaForSteppable", registry);
      maxAngleForSteppable.set(maxNormalAngleFromVertical);
      minimumAreaForSteppable.set(minimumAreaToConsider);

      constraintRegionChanged = new YoBoolean("constraintRegionChanged", registry);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean("switchPlanarRegionConstraintsAutomatically", registry);
      switchPlanarRegionConstraintsAutomatically.set(true);

      yoConvexHullConstraint = new YoFrameConvexPolygon2D("convexHullConstraint", "", worldFrame, 12, registry);
      yoConvexHullConstraintInControlPlane = new YoFrameConvexPolygon2D("convexHullConstraintInControlPlane", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ConvexHullConstraintInControlPlane",
                                                                                       yoConvexHullConstraintInControlPlane,
                                                                                       Color.PINK,
                                                                                       false);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
      }
   }

   public void setPlanarRegions(java.util.List<PlanarRegion> planarRegions)
   {
      steppableRegions = planarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());
   }

   public void setCaptureRegion(FrameConvexPolygon2DReadOnly captureRegion)
   {
      this.captureRegion.setIncludingFrame(captureRegion);
   }

   public void setOmega0(double omega)
   {
      icpControlPlane.setOmega0(omega);
   }

   public void reset()
   {
      planarRegionToConstrainTo = null;
      yoConvexHullConstraint.clear();
      yoConvexHullConstraintInControlPlane.clear();
   }

   private void computeShrunkAndProjectedConvexHulls(PlanarRegion planarRegion)
   {
      yoConvexHullConstraint.set(planarRegion.getConvexHull());
      yoConvexHullConstraint.applyTransform(planarRegion.getTransformToWorld(), false);

      icpControlPlane.projectConvexHullOntoControlPlane(planarRegion.getConvexHull(), planarRegion.getTransformToWorld(), yoConvexHullConstraintInControlPlane);
   }

   public PlanarRegion updatePlanarRegionConstraintForStep(FramePose3DReadOnly footstepPose)
   {
      constraintRegionChanged.set(false);

      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose.getPosition());

         if (planarRegionToConstrainTo != null)
            computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo);
      }

      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue() && !checkIfCurrentPlanarRegionIsValid(captureRegion))
      {
         PlanarRegion betterRegion = findBestPlanarRegionToStepTo(captureRegion);
         if (betterRegion != null)
         {
            planarRegionToConstrainTo = betterRegion;
            computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      return planarRegionToConstrainTo;
   }

   public boolean constraintRegionChanged()
   {
      return constraintRegionChanged.getBooleanValue();
   }

   public PlanarRegion getConstraintRegion()
   {
      return planarRegionToConstrainTo;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      double angle = planarRegion.getNormal().angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      return PlanarRegionTools.computePlanarRegionArea(planarRegion) > minimumAreaForSteppable.getValue();
   }

   private PlanarRegion findPlanarRegionUnderFoothold(FramePoint3DReadOnly foothold)
   {
      PlanarRegion highestRegionUnderFoot = new PlanarRegion();
      Point3D projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(foothold, steppableRegions, highestRegionUnderFoot);

      return projectedPoint == null ? null : highestRegionUnderFoot;
   }

   private boolean checkIfCurrentPlanarRegionIsValid(FrameConvexPolygon2DReadOnly captureRegion)
   {
      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, yoConvexHullConstraintInControlPlane);

      return intersectionArea > minimumIntersectionForSearch;
   }

   private PlanarRegion findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly captureRegion)
   {
      double maxArea = 0.0;
      PlanarRegion activePlanarRegion = null;

      for (int regionIndex = 0; regionIndex < steppableRegions.size(); regionIndex++)
      {
         PlanarRegion planarRegion = steppableRegions.get(regionIndex);

         double intersectionArea = findIntersectionAreaWithCaptureRegion(captureRegion, planarRegion);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = planarRegion;
         }
      }

      return activePlanarRegion;
   }

   private double findIntersectionAreaWithCaptureRegion(FrameConvexPolygon2DReadOnly captureRegionInControlPlane, PlanarRegion planarRegion)
   {
      if (!planarRegion.isPolygonIntersecting(captureRegionInControlPlane))
         return 0.0;

      double intersectionArea = 0.0;
      for (ConvexPolygon2DReadOnly convexPolygon : planarRegion.getConvexPolygons())
      {
         icpControlPlane.projectConvexHullOntoControlPlane(convexPolygon, planarRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
         intersectionArea += convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, convexHullConstraintInControlPlane);
      }

      return intersectionArea;
   }

}
