package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPOptimizationReachabilityConstraintHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int numberOfVertices = 5;

   private final SideDependentList<List<YoFramePoint2D>> reachabilityVertices = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> reachabilityPolygons = new SideDependentList<>();

   private final YoFrameConvexPolygon2D reachabilityPolygon;
   private final YoFrameLineSegment2D motionLimitLine;
   private final YoFrameLineSegment2D adjustmentLineSegment;

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;

   public ICPOptimizationReachabilityConstraintHandler(SideDependentList<ReferenceFrame> soleZUpFrames, ICPOptimizationParameters icpOptimizationParameters,
                                                       SteppingParameters steppingParameters, String yoNamePrefix, boolean visualize,
                                                       YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      lengthLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityLength", registry, steppingParameters.getMaxStepLength());
      lengthBackLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityBackwardLength", registry, steppingParameters.getMaxBackwardStepLength());
      innerLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityWidth", registry, steppingParameters.getMinStepWidth());
      outerLimit = new DoubleParameter(yoNamePrefix + "MinReachabilityWidth", registry, steppingParameters.getMaxStepWidth());


      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame supportSoleFrame = soleZUpFrames.get(robotSide);

         YoInteger yoNumberOfReachabilityVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfReachabilityVertices", registry);
         yoNumberOfReachabilityVertices.set(numberOfVertices);

         String prefix = yoNamePrefix + robotSide.getSideNameFirstLetter();

         List<YoFramePoint2D> reachabilityVertices = new ArrayList<>();
         for (int i = 0; i < yoNumberOfReachabilityVertices.getValue(); i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "ReachabilityVertex" + i, supportSoleFrame, registry);
            reachabilityVertices.add(vertex);
         }
         YoFrameConvexPolygon2D reachabilityPolygon = new YoFrameConvexPolygon2D(reachabilityVertices, yoNumberOfReachabilityVertices, supportSoleFrame);

         this.reachabilityVertices.put(robotSide, reachabilityVertices);
         this.reachabilityPolygons.put(robotSide, reachabilityPolygon);

      }

      reachabilityPolygon = new YoFrameConvexPolygon2D(yoNamePrefix + "ReachabilityRegion", "", worldFrame, 12, registry);
      motionLimitLine = new YoFrameLineSegment2D(yoNamePrefix + "AdjustmentThresholdSegment", "", worldFrame, registry);
      adjustmentLineSegment = new YoFrameLineSegment2D(yoNamePrefix + "AdjustmentLineSegment", "", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ReachabilityRegionViz", reachabilityPolygon, Color.BLUE, false);
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

   /**
    * Initializes the reachability constraint for the double support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    */
   public FrameConvexPolygon2DReadOnly initializeReachabilityConstraintForDoubleSupport()
   {
      reachabilityPolygon.clear();
      motionLimitLine.setToNaN();
      adjustmentLineSegment.setToNaN();

      return null;
   }

   /**
    * Initializes the reachability constraint for the single support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    *
    * @param  supportSide the current support side of the robot
    */
   public FrameConvexPolygon2DReadOnly initializeReachabilityConstraintForSingleSupport(RobotSide supportSide, FramePose3DReadOnly footstepPose)
   {
      reachabilityPolygon.setMatchingFrame(getReachabilityPolygon(supportSide), false);
      return reachabilityPolygon;
   }

   private FrameConvexPolygon2DReadOnly getReachabilityPolygon(RobotSide supportSide)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(supportSide);
      YoFrameConvexPolygon2D polygon = reachabilityPolygons.get(supportSide);

      // create an ellipsoid around the center of the forward and backward reachable limits
      double xRadius = 0.5 * (lengthLimit.getValue() + lengthBackLimit.getValue());
      double yRadius = outerLimit.getValue() - innerLimit.getValue();
      double centerX = lengthLimit.getValue() - xRadius;
      double centerY = innerLimit.getValue();

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++)
      {
         double angle = Math.PI * vertexIdx / (vertices.size() - 1);
         double x = centerX + xRadius * Math.cos(angle);
         double y = centerY + yRadius * Math.sin(angle);
         vertices.get(vertexIdx).set(x, supportSide.negateIfLeftSide(y));
      }

      polygon.notifyVerticesChanged();
      polygon.update();

      return polygon;
   }

   private final FramePoint2D adjustedLocation = new FramePoint2D();
   private final FramePoint2D referenceLocation = new FramePoint2D();
   private final FrameVector2D adjustmentDirection = new FrameVector2D();
   private final FrameLine2D motionLine = new FrameLine2D();

   private final FrameConvexPolygonWithLineIntersector2d lineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   public void updateReachabilityBasedOnAdjustment(FramePose3DReadOnly upcomingFootstep, FixedFramePoint2DBasics footstepSolution, boolean wasAdjusted)
   {
      if (!wasAdjusted)
         return;

      referenceLocation.setIncludingFrame(upcomingFootstep.getPosition());
      adjustedLocation.setIncludingFrame(footstepSolution);
      referenceLocation.changeFrame(worldFrame);
      adjustedLocation.changeFrame(worldFrame);

      adjustmentDirection.sub(adjustedLocation, referenceLocation);
      EuclidGeometryTools.perpendicularVector2D(adjustmentDirection, adjustmentDirection);

      motionLine.getPoint().set(adjustedLocation);
      motionLine.getDirection().set(adjustmentDirection);

      reachabilityPolygon.update();
      ConvexPolygonTools.cutPolygonWithLine(motionLine, reachabilityPolygon, lineIntersector2d, RobotSide.LEFT);

      adjustmentLineSegment.set(referenceLocation, adjustedLocation);
      motionLimitLine.set(lineIntersector2d.getIntersectionPointOne(), lineIntersector2d.getIntersectionPointTwo());
   }

   /**
    * Get the polygon that describes the reachable region for the step position.
    */
   public FrameConvexPolygon2DReadOnly updateReachabilityConstraint()
   {
      reachabilityPolygon.update();

      return reachabilityPolygon;
   }
}
