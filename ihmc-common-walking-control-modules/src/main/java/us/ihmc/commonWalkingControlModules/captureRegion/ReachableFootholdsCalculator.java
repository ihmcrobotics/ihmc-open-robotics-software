package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class ReachableFootholdsCalculator
{
   private static final int numberOfVertices = 5;

   private final PoseReferenceFrame rotatedSoleFrame = new PoseReferenceFrame("rotatedSoleFrame", ReferenceFrame.getWorldFrame());

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;

   public ReachableFootholdsCalculator(DoubleProvider maxStepLength,
                                       DoubleProvider maxBackwardStepLength,
                                       DoubleProvider minStepWidth,
                                       DoubleProvider maxStepWidth)
   {
      this.lengthLimit = maxStepLength;
      this.lengthBackLimit = maxBackwardStepLength;
      this.innerLimit = minStepWidth;
      this.outerLimit = maxStepWidth;
      //      this.midFootAnkleXOffset = midFootAnkleXOffset;
   }

   public void calculateReachableRegion(RobotSide swingSide,
                                        FramePoint3DReadOnly stancePosition,
                                        FrameQuaternionReadOnly stanceOrientation,
                                        FrameConvexPolygon2DBasics reachableRegionToPack)
   {
      rotatedSoleFrame.setPoseAndUpdate(stancePosition, stanceOrientation);
      calculateReachableRegion(swingSide, reachableRegionToPack);
   }

   private void calculateReachableRegion(RobotSide swingSide, FrameConvexPolygon2DBasics reachableRegionToPack)
   {
      double sign = swingSide.negateIfRightSide(1.0);

      reachableRegionToPack.clear(rotatedSoleFrame);

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

         reachableRegionToPack.addVertex(x, sign * y);
      }

      reachableRegionToPack.update();
      reachableRegionToPack.changeFrameAndProjectToXYPlane(worldFrame);
   }
}
