package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ReachabilityConstraintCalculator
{
   private static final double distanceInsideRegion = 0.04;
   private static final int numberOfVertices = 5;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble lengthLimit;
   private final YoDouble lengthBackLimit;
   private final YoDouble innerLimit;
   private final YoDouble outerLimit;

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   public ReachabilityConstraintCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                           SteppingParameters steppingParameters,
                                           YoRegistry parentRegistry)
   {
      this(soleZUpFrames,
           steppingParameters.getFootLength(),
           steppingParameters.getFootWidth(),
           steppingParameters.getMaxStepLength(),
           steppingParameters.getMaxBackwardStepLength(),
           steppingParameters.getMinStepWidth(),
           steppingParameters.getMaxStepWidth(),
           parentRegistry);
   }

   public ReachabilityConstraintCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                           double footLength,
                                           double footWidth,
                                           double maxStepLength,
                                           double maxBackwardStepLength,
                                           double minStepWidth,
                                           double maxStepWidth,
                                           YoRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;

      lengthLimit = new YoDouble("MaxReachabilityLength", registry);
      lengthBackLimit = new YoDouble("MaxReachabilityBackwardLength", registry);
      innerLimit = new YoDouble("MinReachabilityWidth", registry);
      outerLimit = new YoDouble("MaxReachabilityWidth", registry);

      lengthLimit.set(maxStepLength + 0.5 * footLength + distanceInsideRegion);
      lengthBackLimit.set(maxBackwardStepLength + 0.5 * footLength + distanceInsideRegion);
      innerLimit.set(minStepWidth - 0.5 * footWidth - distanceInsideRegion);
      outerLimit.set(maxStepWidth + 0.5 * footWidth + distanceInsideRegion);

      parentRegistry.addChild(registry);
   }

   public FrameConvexPolygon2DReadOnly getReachabilityPolygon(RobotSide supportSide)
   {
      FrameConvexPolygon2DBasics polygon = new FrameConvexPolygon2D(soleZUpFrames.get(supportSide));

      // create an ellipsoid around the center of the forward and backward reachable limits
      double xRadius = 0.5 * (lengthLimit.getValue() + lengthBackLimit.getValue());
      double yRadius = outerLimit.getValue() - innerLimit.getValue();
      double centerX = lengthLimit.getValue() - xRadius;
      double centerY = innerLimit.getValue();

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < numberOfVertices; vertexIdx++)
      {
         double angle = Math.PI * vertexIdx / (numberOfVertices - 1);
         double x = centerX + xRadius * Math.cos(angle);
         double y = centerY + yRadius * Math.sin(angle);
         polygon.addVertex(x, supportSide.negateIfLeftSide(y));
      }

      polygon.update();
      polygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      return polygon;
   }
}
