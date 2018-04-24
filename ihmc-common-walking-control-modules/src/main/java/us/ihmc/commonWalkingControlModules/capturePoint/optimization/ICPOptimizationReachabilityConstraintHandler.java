package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPOptimizationReachabilityConstraintHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<List<YoFramePoint2D>> reachabilityVertices = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> reachabilityPolygons = new SideDependentList<>();

   private final YoFrameConvexPolygon2D contractedReachabilityPolygon;
   private final YoFrameLineSegment2D motionLimitLine;
   private final YoFrameLineSegment2D adjustmentLineSegment;

   private final DoubleProvider forwardLimit;
   private final DoubleProvider backwardLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;

   public ICPOptimizationReachabilityConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters,
                                                       String yoNamePrefix, boolean visualize, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      forwardLimit = new DoubleParameter(yoNamePrefix + "ForwardReachabilityLimit", registry, icpOptimizationParameters.getForwardReachabilityLimit());
      backwardLimit = new DoubleParameter(yoNamePrefix + "BackwardReachabilityLimit", registry, icpOptimizationParameters.getBackwardReachabilityLimit());
      innerLimit = new DoubleParameter(yoNamePrefix + "LateralReachabilityInnerLimit", registry, icpOptimizationParameters.getLateralReachabilityInnerLimit());
      outerLimit = new DoubleParameter(yoNamePrefix + "LateralReachabilityOuterLimit", registry, icpOptimizationParameters.getLateralReachabilityOuterLimit());

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = bipedSupportPolygons.getSoleZUpFrames().get(robotSide);

         List<YoFramePoint2D> reachabilityVertices = new ArrayList<>();
         YoFramePoint2D frontInsidePoint = new YoFramePoint2D(yoNamePrefix + robotSide.getSideNameFirstLetter() + "FrontInsidePoint", soleFrame, registry);
         YoFramePoint2D frontOutsidePoint = new YoFramePoint2D(yoNamePrefix + robotSide.getSideNameFirstLetter() + "FrontOutsidePoint", soleFrame, registry);
         YoFramePoint2D backInsidePoint = new YoFramePoint2D(yoNamePrefix + robotSide.getSideNameFirstLetter() + "BackInsidePoint", soleFrame, registry);
         YoFramePoint2D backOutsidePoint = new YoFramePoint2D(yoNamePrefix + robotSide.getSideNameFirstLetter() + "BackOutsidePoint", soleFrame, registry);

         YoInteger numberOfVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfReachabilityVertices", registry);
         numberOfVertices.set(4);

         reachabilityVertices.add(frontInsidePoint);
         reachabilityVertices.add(frontOutsidePoint);
         reachabilityVertices.add(backInsidePoint);
         reachabilityVertices.add(backOutsidePoint);

         YoFrameConvexPolygon2D reachabilityPolygon = new YoFrameConvexPolygon2D(reachabilityVertices, numberOfVertices, soleFrame);

         this.reachabilityVertices.put(robotSide, reachabilityVertices);
         this.reachabilityPolygons.put(robotSide, reachabilityPolygon);
      }

      contractedReachabilityPolygon = new YoFrameConvexPolygon2D(yoNamePrefix + "ReachabilityRegion", "", worldFrame, 12, registry);
      motionLimitLine = new YoFrameLineSegment2D(yoNamePrefix + "AdjustmentThresholdSegment", "", worldFrame, registry);
      adjustmentLineSegment = new YoFrameLineSegment2D(yoNamePrefix + "AdjustmentLineSegment", "", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ReachabilityRegionViz", contractedReachabilityPolygon, Color.BLUE, false);
         YoArtifactLineSegment2d adjustmentGraphic = new YoArtifactLineSegment2d("AdjustmentViz", adjustmentLineSegment, Color.GREEN);
         YoArtifactLineSegment2d adjustmentClippingGraphic = new YoArtifactLineSegment2d("AdjustmentClippingViz", motionLimitLine, Color.RED);

         reachabilityGraphic.setVisible(visualize);
         adjustmentGraphic.setVisible(visualize);
         adjustmentClippingGraphic.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), adjustmentGraphic);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), adjustmentClippingGraphic);
      }
   }

   public void initializeReachabilityConstraintForDoubleSupport(ICPOptimizationQPSolver solver)
   {
      contractedReachabilityPolygon.clear();
      motionLimitLine.setToNaN();
      adjustmentLineSegment.setToNaN();
      solver.resetReachabilityConstraint();
      solver.resetPlanarRegionConstraint();
   }

   public void initializeReachabilityConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationQPSolver solver)
   {
      solver.resetReachabilityConstraint();

      YoFrameConvexPolygon2D reachabilityPolygon = getReachabilityPolygon(supportSide);
      contractedReachabilityPolygon.setMatchingFrame(reachabilityPolygon, false);
      contractedReachabilityPolygon.update();
      solver.addReachabilityPolygon(contractedReachabilityPolygon);
      solver.resetPlanarRegionConstraint();
   }

   private YoFrameConvexPolygon2D getReachabilityPolygon(RobotSide supportSide)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(supportSide);
      YoFrameConvexPolygon2D polygon = reachabilityPolygons.get(supportSide);

      double forwardLimit = this.forwardLimit.getValue();
      double backwardLimit = this.backwardLimit.getValue();
      double innerLimit = supportSide.negateIfLeftSide(this.innerLimit.getValue());
      double outerLimit = supportSide.negateIfLeftSide(this.outerLimit.getValue());

      vertices.get(0).set(forwardLimit, innerLimit);
      vertices.get(1).set(forwardLimit, outerLimit);
      vertices.get(2).set(backwardLimit, innerLimit);
      vertices.get(3).set(backwardLimit, outerLimit);

      polygon.notifyVerticesChanged();
      polygon.update();

      return polygon;
   }

   private final FramePoint2D adjustedLocation = new FramePoint2D();
   private final FramePoint2D referenceLocation = new FramePoint2D();
   private final FrameVector2D adjustmentDirection = new FrameVector2D();
   private final FrameLine2D motionLine = new FrameLine2D();

   private final FrameConvexPolygonWithLineIntersector2d lineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   public void updateReachabilityBasedOnAdjustment(Footstep upcomingFootstep, FixedFramePoint2DBasics footstepSolution, boolean wasAdjusted)
   {
      if (!wasAdjusted)
         return;

      upcomingFootstep.getPosition2d(referenceLocation);
      adjustedLocation.setIncludingFrame(footstepSolution);
      referenceLocation.changeFrame(worldFrame);
      adjustedLocation.changeFrame(worldFrame);

      adjustmentDirection.sub(adjustedLocation, referenceLocation);
      EuclidGeometryTools.perpendicularVector2D(adjustmentDirection, adjustmentDirection);

      motionLine.setPoint(adjustedLocation);
      motionLine.setDirection(adjustmentDirection);

      contractedReachabilityPolygon.update();
      ConvexPolygonTools.cutPolygonWithLine(motionLine, contractedReachabilityPolygon, lineIntersector2d, RobotSide.LEFT);

      adjustmentLineSegment.set(referenceLocation, adjustedLocation);
      motionLimitLine.set(lineIntersector2d.getIntersectionPointOne(), lineIntersector2d.getIntersectionPointTwo());
   }

   public void updateReachabilityConstraint(ICPOptimizationQPSolver solver)
   {
      solver.resetReachabilityConstraint();
      contractedReachabilityPolygon.update();
      solver.addReachabilityPolygon(contractedReachabilityPolygon);
   }
}
