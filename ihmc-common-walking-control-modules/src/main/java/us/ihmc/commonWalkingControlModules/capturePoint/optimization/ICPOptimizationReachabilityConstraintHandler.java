package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPOptimizationReachabilityConstraintHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SideDependentList<YoFrameConvexPolygon2d> reachabilityPolygons = new SideDependentList<>();

   private final YoFrameConvexPolygon2d contractedReachabilityPolygon;
   private final YoFrameLineSegment2d motionLimitLine;
   private final YoFrameLineSegment2d adjustmentLineSegment;

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

         ArrayList<YoFramePoint2d> reachabilityVertices = new ArrayList<>();
         YoFramePoint2d frontInsidePoint = new YoFramePoint2d(forwardLimit, innerLimit, soleFrame);
         YoFramePoint2d frontOutsidePoint = new YoFramePoint2d(forwardLimit, outerLimit, soleFrame);
         YoFramePoint2d backInsidePoint = new YoFramePoint2d(backwardLimit, innerLimit, soleFrame);
         YoFramePoint2d backOutsidePoint = new YoFramePoint2d(backwardLimit, outerLimit, soleFrame);

         YoInteger numberOfVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfReachabilityVertices", registry);
         numberOfVertices.set(4);

         reachabilityVertices.add(frontInsidePoint);
         reachabilityVertices.add(frontOutsidePoint);
         reachabilityVertices.add(backInsidePoint);
         reachabilityVertices.add(backOutsidePoint);

         YoFrameConvexPolygon2d reachabilityPolygon = new YoFrameConvexPolygon2d(reachabilityVertices, numberOfVertices, soleFrame);

         reachabilityPolygons.put(robotSide, reachabilityPolygon);
      }

      contractedReachabilityPolygon = new YoFrameConvexPolygon2d(yoNamePrefix + "ReachabilityRegion", "", worldFrame, 12, registry);
      motionLimitLine = new YoFrameLineSegment2d(yoNamePrefix + "AdjustmentThresholdSegment", "", worldFrame, registry);
      adjustmentLineSegment = new YoFrameLineSegment2d(yoNamePrefix + "AdjustmentLineSegment", "", worldFrame, registry);

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
      contractedReachabilityPolygon.clearAndHide();
      motionLimitLine.setToNaN();
      adjustmentLineSegment.setToNaN();
      solver.resetReachabilityConstraint();
   }

   public void initializeReachabilityConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationQPSolver solver)
   {
      solver.resetReachabilityConstraint();

      FrameConvexPolygon2d reachabilityPolygon = reachabilityPolygons.get(supportSide).getFrameConvexPolygon2d();
      reachabilityPolygon.changeFrame(ReferenceFrame.getWorldFrame());
      contractedReachabilityPolygon.setConvexPolygon2d(reachabilityPolygon.getConvexPolygon2d());

      FrameConvexPolygon2d polygon2d = contractedReachabilityPolygon.getFrameConvexPolygon2d();
      polygon2d.update();
      solver.addReachabilityPolygon(polygon2d);
   }

   private final FramePoint2D adjustedLocation = new FramePoint2D();
   private final FramePoint2D referenceLocation = new FramePoint2D();
   private final FrameVector2D adjustmentDirection = new FrameVector2D();
   private final FrameLine2d motionLine = new FrameLine2d();

   private final FrameConvexPolygonWithLineIntersector2d lineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   public void updateReachabilityBasedOnAdjustment(Footstep upcomingFootstep, FramePoint2D footstepSolution, boolean wasAdjusted)
   {
      if (!wasAdjusted)
         return;

      upcomingFootstep.getPosition2d(referenceLocation);
      adjustedLocation.setIncludingFrame(footstepSolution);
      referenceLocation.changeFrame(worldFrame);
      adjustedLocation.changeFrame(worldFrame);

      adjustmentDirection.set(adjustedLocation);
      adjustmentDirection.sub(referenceLocation);
      EuclidGeometryTools.perpendicularVector2D(adjustmentDirection, adjustmentDirection);

      motionLine.setPoint(adjustedLocation);
      motionLine.setVector(adjustmentDirection);

      FrameConvexPolygon2d polygon2d = contractedReachabilityPolygon.getFrameConvexPolygon2d();
      polygon2d.update();
      ConvexPolygonTools.cutPolygonWithLine(motionLine, polygon2d, lineIntersector2d, RobotSide.LEFT);

      adjustmentLineSegment.set(referenceLocation, adjustedLocation);
      motionLimitLine.set(lineIntersector2d.getIntersectionPointOne(), lineIntersector2d.getIntersectionPointTwo());

      contractedReachabilityPolygon.setConvexPolygon2d(polygon2d.getConvexPolygon2d());
   }

   public void updateReachabilityConstraint(ICPOptimizationQPSolver solver)
   {
      solver.resetReachabilityConstraint();
      FrameConvexPolygon2d polygon2d = contractedReachabilityPolygon.getFrameConvexPolygon2d();
      polygon2d.update();
      solver.addReachabilityPolygon(polygon2d);
   }
}
