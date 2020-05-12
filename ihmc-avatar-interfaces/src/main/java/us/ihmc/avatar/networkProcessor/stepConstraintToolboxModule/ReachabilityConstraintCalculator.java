package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ReachabilityConstraintCalculator
{
   private static final int numberOfVertices = 5;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   public ReachabilityConstraintCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                           SteppingParameters steppingParameters,
                                           YoVariableRegistry parentRegistry)
   {
      this(soleZUpFrames,
           steppingParameters.getMaxStepLength(),
           steppingParameters.getMaxBackwardStepLength(),
           steppingParameters.getMinStepWidth(),
           steppingParameters.getMaxStepWidth(),
           parentRegistry);
   }

   public ReachabilityConstraintCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                           double maxStepLength,
                                           double maxBackwardStepLength,
                                           double minStepWidth,
                                           double maxStepWidth,
                                           YoVariableRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;

      lengthLimit = new DoubleParameter("MaxReachabilityLength", registry, maxStepLength);
      lengthBackLimit = new DoubleParameter("MaxReachabilityBackwardLength", registry, maxBackwardStepLength);
      innerLimit = new DoubleParameter("MinReachabilityWidth", registry, minStepWidth);
      outerLimit = new DoubleParameter("MaxReachabilityWidth", registry, maxStepWidth);

      parentRegistry.addChild(registry);
   }

   private FrameConvexPolygon2DReadOnly getReachabilityPolygon(RobotSide supportSide)
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

      return polygon;
   }
}
