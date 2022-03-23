package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class StepAdjustmentReachabilityConstraint
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int numberOfVertices = 9;
   private static final double SUFFICIENTLY_LARGE = 5.0;

   private final SideDependentList<List<YoFramePoint2D>> reachabilityVertices = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> reachabilityPolygons = new SideDependentList<>();

   private final SideDependentList<List<YoFramePoint2D>> adjustmentVertices = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> stepFrames = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> adjustmentPolygons = new SideDependentList<>();

   private final FixedFrameConvexPolygon2DBasics adjustmentPolygon = new FrameConvexPolygon2D(worldFrame);
   private final FixedFrameConvexPolygon2DBasics reachabilityPolygon = new FrameConvexPolygon2D(worldFrame);

   private final YoFrameConvexPolygon2D contractedReachabilityPolygon;

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;
   private final DoubleProvider inPlaceWidth;

   private final DoubleProvider forwardAdjustmentLimit;
   private final DoubleProvider backwardAdjustmentLimit;
   private final DoubleProvider inwardAdjustmentLimit;
   private final DoubleProvider outwardAdjustmentLimit;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();

   public StepAdjustmentReachabilityConstraint(SideDependentList<? extends ReferenceFrame> soleZUpFrames, ICPOptimizationParameters icpOptimizationParameters,
                                               SteppingParameters steppingParameters, String yoNamePrefix, boolean visualize,
                                               YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(soleZUpFrames,
           icpOptimizationParameters,
           new DoubleParameter(yoNamePrefix + "MaxReachabilityLength", registry, steppingParameters.getMaxStepLength()),
           new DoubleParameter(yoNamePrefix + "MaxReachabilityBackwardLength", registry, steppingParameters.getMaxBackwardStepLength()),
           new DoubleParameter(yoNamePrefix + "MinReachabilityWidth", registry, steppingParameters.getMinStepWidth()),
           new DoubleParameter(yoNamePrefix + "MaxReachabilityWidth", registry, steppingParameters.getMaxStepWidth()),
           new DoubleParameter(yoNamePrefix + "InPlaceWidth", registry, steppingParameters.getInPlaceWidth()),
           yoNamePrefix,
           visualize,
           registry,
           yoGraphicsListRegistry);
   }

   public StepAdjustmentReachabilityConstraint(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                               ICPOptimizationParameters icpOptimizationParameters,
                                               DoubleProvider lengthLimit,
                                               DoubleProvider lengthBackLimit,
                                               DoubleProvider innerLimit,
                                               DoubleProvider outerLimit,
                                               DoubleProvider inPlaceWidth,
                                               String yoNamePrefix,
                                               boolean visualize,
                                               YoRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(soleZUpFrames,
           lengthLimit,
           lengthBackLimit,
           innerLimit,
           outerLimit,
           inPlaceWidth,
           new DoubleParameter(yoNamePrefix + "ForwardAdjustmentLimit", registry,
                               Math.min(SUFFICIENTLY_LARGE, icpOptimizationParameters.getMaximumStepAdjustmentForward())),
           new DoubleParameter(yoNamePrefix + "BackwardAdjustmentLimit", registry,
                               Math.min(SUFFICIENTLY_LARGE, icpOptimizationParameters.getMaximumStepAdjustmentBackward())),
           new DoubleParameter(yoNamePrefix + "InwardAdjustmentLimit", registry,
                               Math.min(SUFFICIENTLY_LARGE, icpOptimizationParameters.getMaximumStepAdjustmentInward())),
           new DoubleParameter(yoNamePrefix + "OutwardAdjustmentLimit", registry,
                               Math.min(SUFFICIENTLY_LARGE, icpOptimizationParameters.getMaximumStepAdjustmentOutward())),
           yoNamePrefix,
           visualize,
           registry,
           yoGraphicsListRegistry);
   }

   public StepAdjustmentReachabilityConstraint(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                               DoubleProvider lengthLimit,
                                               DoubleProvider lengthBackLimit,
                                               DoubleProvider innerLimit,
                                               DoubleProvider outerLimit,
                                               DoubleProvider inPlaceWidth,
                                               String yoNamePrefix,
                                               boolean visualize,
                                               YoRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(soleZUpFrames, lengthLimit, lengthBackLimit, innerLimit, outerLimit, inPlaceWidth, null, null, null, null, yoNamePrefix, visualize, registry, yoGraphicsListRegistry);
   }

   public StepAdjustmentReachabilityConstraint(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                               DoubleProvider lengthLimit,
                                               DoubleProvider lengthBackLimit,
                                               DoubleProvider innerLimit,
                                               DoubleProvider outerLimit,
                                               DoubleProvider inPlaceWidth,
                                               DoubleProvider forwardAdjustmentLimit,
                                               DoubleProvider backwardAdjustmentLimit,
                                               DoubleProvider inwardAdjustmentLimit,
                                               DoubleProvider outwardAdjustmentLimit,
                                               String yoNamePrefix,
                                               boolean visualize,
                                               YoRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.lengthLimit = lengthLimit;
      this.lengthBackLimit = lengthBackLimit;
      this.innerLimit = innerLimit;
      this.outerLimit = outerLimit;
      this.inPlaceWidth = inPlaceWidth;
      this.forwardAdjustmentLimit = forwardAdjustmentLimit;
      this.backwardAdjustmentLimit = backwardAdjustmentLimit;
      this.inwardAdjustmentLimit = inwardAdjustmentLimit;
      this.outwardAdjustmentLimit = outwardAdjustmentLimit;


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

         PoseReferenceFrame adjustmentFrame = new PoseReferenceFrame(prefix + "AdjustmentFrame", worldFrame);

         YoInteger yoNumberOfAdjustmentVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfAdjustmentVertices", registry);
         yoNumberOfAdjustmentVertices.set(4);

         List<YoFramePoint2D> adjustmentVertices = new ArrayList<>();
         for (int i = 0; i < 4; i++)
         {
            YoFramePoint2D adjustmentVertex = new YoFramePoint2D(prefix + "AdjustmentVertex" + i, adjustmentFrame, registry);
            adjustmentVertices.add(adjustmentVertex);
         }

         YoFrameConvexPolygon2D adjustmentPolygon = new YoFrameConvexPolygon2D(adjustmentVertices, yoNumberOfAdjustmentVertices, adjustmentFrame);

         this.stepFrames.put(robotSide, adjustmentFrame);
         this.adjustmentVertices.put(robotSide, adjustmentVertices);
         this.adjustmentPolygons.put(robotSide, adjustmentPolygon);

      }

      contractedReachabilityPolygon = new YoFrameConvexPolygon2D(yoNamePrefix + "ReachabilityRegion", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ReachabilityRegionViz", contractedReachabilityPolygon, Color.BLUE, false);

         reachabilityGraphic.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
      }
   }

   /**
    * Initializes the reachability constraint for the double support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    */
   public void reset()
   {
      contractedReachabilityPolygon.clear();
   }

   /**
    * Initializes the reachability constraint for the single support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    *
    * @param  supportSide the current support side of the robot
    */
   public FrameConvexPolygon2DReadOnly initializeReachabilityConstraint(RobotSide supportSide, FramePose3DReadOnly footstepPose)
   {
      reachabilityPolygon.setMatchingFrame(updateReachabilityPolygon(supportSide), false);
      updateReachabilityPolygon(supportSide.getOppositeSide());
      FrameConvexPolygon2DReadOnly adjustmentPolygon = getAdjustmentPolygon(supportSide.getOppositeSide(), footstepPose);

      contractedReachabilityPolygon.checkReferenceFrameMatch(reachabilityPolygon);
      if (adjustmentPolygon != null)
      {
         contractedReachabilityPolygon.checkReferenceFrameMatch(adjustmentPolygon);

         if (polygonTools.computeIntersectionOfPolygons(reachabilityPolygon, adjustmentPolygon, contractedReachabilityPolygon))
            return contractedReachabilityPolygon;

         // there is no intersection, make the reachability only a single point.
         contractedReachabilityPolygon.clear();
         contractedReachabilityPolygon.addVertex(footstepPose.getPosition());
         contractedReachabilityPolygon.update();
      }
      else
      {
         contractedReachabilityPolygon.setMatchingFrame(reachabilityPolygon, false);
      }

      return contractedReachabilityPolygon;
   }

   private FrameConvexPolygon2DReadOnly updateReachabilityPolygon(RobotSide supportSide)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(supportSide);
      YoFrameConvexPolygon2D polygon = reachabilityPolygons.get(supportSide);

      // create an ellipsoid around the center of the forward and backward reachable limits
      double innerRadius = inPlaceWidth.getValue() - innerLimit.getValue();
      double outerRadius = outerLimit.getValue() - inPlaceWidth.getValue();

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++)
      {
         double angle = 2.0 * Math.PI * vertexIdx / (vertices.size() - 1);
         double x, y;
         if (angle < Math.PI / 2.0)
         {
            x = lengthLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - innerRadius * Math.sin(angle));
         }
         else if (angle < Math.PI)
         {
            x = lengthBackLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - innerRadius * Math.sin(angle));
         }
         else if (angle < 1.5 * Math.PI)
         {
            x = lengthBackLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - outerRadius * Math.sin(angle));
         }
         else
         {
            x = lengthLimit.getValue() * Math.cos(angle);
            y = supportSide.negateIfLeftSide(inPlaceWidth.getValue() - outerRadius * Math.sin(angle));
         }

         FixedFramePoint2DBasics vertex = vertices.get(vertexIdx);
         vertex.set(x, y);
      }


      polygon.notifyVerticesChanged();
      polygon.update();

      return polygon;
   }

   private FrameConvexPolygon2DReadOnly getAdjustmentPolygon(RobotSide swingSide, FramePose3DReadOnly footstepPose)
   {
      if (forwardAdjustmentLimit == null || backwardAdjustmentLimit == null || outwardAdjustmentLimit == null || inwardAdjustmentLimit == null)
         return null;

      stepFrames.get(swingSide).setPoseAndUpdate(footstepPose);

      List<YoFramePoint2D> vertices = adjustmentVertices.get(swingSide);
      YoFrameConvexPolygon2D polygon = adjustmentPolygons.get(swingSide);

      vertices.get(0).set(forwardAdjustmentLimit.getValue(), swingSide.negateIfRightSide(outwardAdjustmentLimit.getValue()));
      vertices.get(1).set(forwardAdjustmentLimit.getValue(), swingSide.negateIfLeftSide(inwardAdjustmentLimit.getValue()));
      vertices.get(2).set(-backwardAdjustmentLimit.getValue(), swingSide.negateIfRightSide(outwardAdjustmentLimit.getValue()));
      vertices.get(3).set(-backwardAdjustmentLimit.getValue(), swingSide.negateIfLeftSide(inwardAdjustmentLimit.getValue()));

      polygon.notifyVerticesChanged();
      polygon.update();

      adjustmentPolygon.setMatchingFrame(polygon, false);
      return adjustmentPolygon;
   }

   /**
    * Get the polygon that describes the reachable region for the step position.
    */
   public FrameConvexPolygon2DReadOnly getReachabilityConstraint()
   {
      return contractedReachabilityPolygon;
   }

   public FrameConvexPolygon2DReadOnly getReachabilityPolygonInFootFrame(RobotSide robotSide)
   {
      return reachabilityPolygons.get(robotSide);
   }
}
