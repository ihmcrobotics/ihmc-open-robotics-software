package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class EnvironmentConstraintHandler implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double defaultDesiredDistanceInside = 0.06;
   private static final boolean defaultUsePredictedContactPoints = false;
   private static final double defaultMaxConcaveEstimateRatio = 1.1;

   private final DoubleProvider desiredDistanceInsideConstraint;
   private final BooleanProvider usePredictedContactPoints;
   private final DoubleProvider maxConcaveEstimateRatio;

   private final YoBoolean foundSolution;

   private final YoBoolean isEnvironmentConstraintValid;

   private final YoConstraintOptimizerParameters parameters;

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final YoFrameConvexPolygon2D yoConvexHullConstraint;
   private final FrameConvexPolygon2D convexHullConstraint = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D reachabilityRegionInConstraintPlane = new FrameConvexPolygon2D();

   private final List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

   private final FramePoint3D projectedReachablePoint = new FramePoint3D();

   private final ConvexPolygon2D footstepPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform footOrientationTransform = new RigidBodyTransform();
   private final FramePoint2D stepXY = new FramePoint2D();

   private final CapturabilityBasedPlanarRegionDecider planarRegionDecider;
   private final ConvexStepConstraintOptimizer stepConstraintOptimizer;

   public EnvironmentConstraintHandler(ICPControlPlane icpControlPlane,
                                       SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                       BooleanProvider useICPControlPlaneInStepAdjustment,
                                       String yoNamePrefix,
                                       YoRegistry registry,
                                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactableFeet = contactableFeet;

      planarRegionDecider = new CapturabilityBasedPlanarRegionDecider(icpControlPlane, useICPControlPlaneInStepAdjustment, registry, null);
      stepConstraintOptimizer = new ConvexStepConstraintOptimizer(registry);
      parameters = new YoConstraintOptimizerParameters(registry);

      desiredDistanceInsideConstraint = new DoubleParameter("desiredDistanceInsideEnvironmentConstraint", registry, defaultDesiredDistanceInside);
      usePredictedContactPoints = new BooleanParameter("usePredictedContactPointsInStep", registry, defaultUsePredictedContactPoints);
      maxConcaveEstimateRatio = new DoubleParameter("maxConcaveEstimateRatio", registry, defaultMaxConcaveEstimateRatio);

      isEnvironmentConstraintValid = new YoBoolean("isEnvironmentConstraintValid", registry);

      yoConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraint", "", worldFrame, 12, registry);
      foundSolution = new YoBoolean("foundSolutionToStepConstraint", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("Environmental Convex Hull Constraint", yoConvexHullConstraint, Color.RED, false);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
      }
   }

   public void setStepConstraintRegions(List<StepConstraintRegion> stepConstraintRegions)
   {
      reset();
      planarRegionDecider.setConstraintRegions(stepConstraintRegions);
   }

   public boolean hasStepConstraintRegion()
   {
      return planarRegionDecider.getConstraintRegion() != null && planarRegionDecider.isStepFarEnoughInsideToIgnoreConstraint();
   }

   public void reset()
   {
      foundSolution.set(false);
      stepConstraintRegions.clear();
      yoConvexHullConstraint.clear();
      convexHullConstraint.clearAndUpdate();
      isEnvironmentConstraintValid.set(false);
      stepConstraintOptimizer.reset();
      planarRegionDecider.reset();
   }

   public void setReachabilityRegion(FrameConvexPolygon2DReadOnly reachabilityRegion)
   {
      this.reachabilityRegionInConstraintPlane.setIncludingFrame(reachabilityRegion);
   }

   public void updateActiveConstraintRegionToUse(FramePose3DReadOnly footstepPose, FrameConvexPolygon2DReadOnly captureRegion)
   {
      planarRegionDecider.setCaptureRegion(captureRegion);
      planarRegionDecider.updatePlanarRegionConstraintForStep(footstepPose, reachabilityRegionInConstraintPlane);
      if (planarRegionDecider.constraintRegionChanged())
         stepConstraintOptimizer.reset();
   }

   private final Point2DBasics centroidToThrowAway = new Point2D();

   public boolean validateConvexityOfPlanarRegion()
   {
      if (stepConstraintRegions.isEmpty())
      {
         isEnvironmentConstraintValid.set(true);
         return isEnvironmentConstraintValid.getBooleanValue();
      }

      StepConstraintRegion stepConstraintRegion = planarRegionDecider.getConstraintRegion();
      double concaveHullArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(stepConstraintRegion.getConcaveHullVertices(),
                                                                                     stepConstraintRegion.getConcaveHullSize(),
                                                                                     true,
                                                                                     centroidToThrowAway);
      double convexHullArea = stepConstraintRegion.getConvexHullInConstraintRegion().getArea();

      isEnvironmentConstraintValid.set(concaveHullArea / convexHullArea < maxConcaveEstimateRatio.getValue());
      return isEnvironmentConstraintValid.getBooleanValue();
   }

   private final FramePose3D originalPose = new FramePose3D();
   private final RigidBodyTransform footstepTransform = new RigidBodyTransform();

   public boolean applyEnvironmentConstraintToFootstep(RobotSide upcomingFootstepSide,
                                                       FixedFramePose3DBasics footstepPoseToPack,
                                                       List<Point2D> predictedContactPoints)
   {
      StepConstraintRegion stepConstraintRegion = planarRegionDecider.getConstraintRegion();

      if (stepConstraintRegion == null)
      {
         foundSolution.set(true);
         return false;
      }

      computeFootstepPolygon(upcomingFootstepSide, predictedContactPoints, footstepPoseToPack.getOrientation());

      convexHullConstraint.set(stepConstraintRegion.getConvexHullInConstraintRegion());
      convexHullConstraint.applyTransform(stepConstraintRegion.getTransformToWorld(), false);

      if (convexHullConstraint.getNumberOfVertices() <= yoConvexHullConstraint.getMaxNumberOfVertices())
         yoConvexHullConstraint.set(convexHullConstraint);
      else
         yoConvexHullConstraint.clearAndUpdate();

      stepXY.set(footstepPoseToPack.getPosition());
      // do a simple orthogonal projection first.
      if (!convexHullConstraint.isPointInside(stepXY))
      {
         convexHullConstraint.orthogonalProjection(stepXY);
         footstepPoseToPack.getPosition().set(stepXY);
      }

      footstepPoseToPack.get(footstepTransform);
      footstepPolygon.applyTransform(footstepTransform, false);
      parameters.setDesiredDistanceInside(desiredDistanceInsideConstraint.getValue());

      RigidBodyTransformReadOnly wiggleTransform = stepConstraintOptimizer.findConstraintTransform(footstepPolygon, convexHullConstraint, parameters);
      originalPose.set(footstepPoseToPack);

      if (wiggleTransform != null)
      {
         foundSolution.set(true);
         footstepPoseToPack.applyTransform(wiggleTransform);
      }
      else
      {
         foundSolution.set(false);
      }

      footstepPoseToPack.getPosition().setZ(stepConstraintRegion.getPlaneZGivenXY(footstepPoseToPack.getX(), footstepPoseToPack.getY()));
      // TODO need to rotate to match the surface normal
      //      footstepPoseToPack.getOrientation().set(stepConstraintRegion.getTransformToWorld().getRotation());

      return originalPose.getPositionDistance(footstepPoseToPack) > 1e-5 || originalPose.getOrientationDistance(footstepPoseToPack) > 1e-5;
   }

   public boolean foundSolution()
   {
      return foundSolution.getBooleanValue();
   }

   private void computeFootstepPolygon(RobotSide upcomingFootstepSide, List<? extends Point2DBasics> predictedContactPoints, Orientation3DReadOnly orientation)
   {
      if (predictedContactPoints.isEmpty() || !usePredictedContactPoints.getValue())
         predictedContactPoints = contactableFeet.get(upcomingFootstepSide).getContactPoints2d();

      footstepPolygon.clear();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         footstepPolygon.addVertex(predictedContactPoints.get(i));
      footstepPolygon.update();

      footOrientationTransform.getRotation().set(orientation);

      footstepPolygon.applyTransform(footOrientationTransform, false);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Environmental Convex Hull Constraint", yoConvexHullConstraint, ColorDefinitions.Red()));
      return group;
   }
}
