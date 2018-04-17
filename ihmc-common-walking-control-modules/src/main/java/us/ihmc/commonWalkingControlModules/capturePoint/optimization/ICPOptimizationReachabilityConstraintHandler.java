package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPOptimizationReachabilityConstraintHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<YoFrameConvexPolygon2D> reachabilityPolygons = new SideDependentList<>();

   private final YoFrameConvexPolygon2D contractedReachabilityPolygon;
   private final YoFrameLineSegment2D motionLimitLine;
   private final YoFrameLineSegment2D adjustmentLineSegment;

   public ICPOptimizationReachabilityConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters,
                                                       String yoNamePrefix, boolean visualize, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoDouble forwardLimit = new YoDouble(yoNamePrefix + "ForwardReachabilityLimit", registry);
      YoDouble backwardLimit = new YoDouble(yoNamePrefix + "BackwardReachabilityLimit", registry);
      forwardLimit.set(icpOptimizationParameters.getForwardReachabilityLimit());
      backwardLimit.set(icpOptimizationParameters.getBackwardReachabilityLimit());

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = bipedSupportPolygons.getSoleZUpFrames().get(robotSide);

         YoDouble innerLimit = new YoDouble(yoNamePrefix + robotSide.getSideNameFirstLetter() + "LateralReachabilityInnerLimit", registry);
         YoDouble outerLimit = new YoDouble(yoNamePrefix + robotSide.getSideNameFirstLetter() + "LateralReachabilityOuterLimit", registry);
         innerLimit.set(robotSide.negateIfLeftSide(icpOptimizationParameters.getLateralReachabilityInnerLimit()));
         outerLimit.set(robotSide.negateIfLeftSide(icpOptimizationParameters.getLateralReachabilityOuterLimit()));

         ArrayList<YoFramePoint2D> reachabilityVertices = new ArrayList<>();
         YoFramePoint2D frontInsidePoint = new YoFramePoint2D(forwardLimit, innerLimit, soleFrame);
         YoFramePoint2D frontOutsidePoint = new YoFramePoint2D(forwardLimit, outerLimit, soleFrame);
         YoFramePoint2D backInsidePoint = new YoFramePoint2D(backwardLimit, innerLimit, soleFrame);
         YoFramePoint2D backOutsidePoint = new YoFramePoint2D(backwardLimit, outerLimit, soleFrame);

         YoInteger numberOfVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfReachabilityVertices", registry);
         numberOfVertices.set(4);

         reachabilityVertices.add(frontInsidePoint);
         reachabilityVertices.add(frontOutsidePoint);
         reachabilityVertices.add(backInsidePoint);
         reachabilityVertices.add(backOutsidePoint);

         YoFrameConvexPolygon2D reachabilityPolygon = new YoFrameConvexPolygon2D(reachabilityVertices, numberOfVertices, soleFrame);

         reachabilityPolygons.put(robotSide, reachabilityPolygon);
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

      reachabilityPolygons.get(supportSide).update();
      contractedReachabilityPolygon.setMatchingFrame(reachabilityPolygons.get(supportSide), false);

      contractedReachabilityPolygon.update();
      solver.addReachabilityPolygon(contractedReachabilityPolygon);
      solver.resetPlanarRegionConstraint();
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
