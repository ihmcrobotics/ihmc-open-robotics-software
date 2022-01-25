package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;

public class InverseCaptureRegionCalculator
{
   private static final int nVertices = 4;
   private final double lengthLimit = 1.0;
   private final double lengthBackLimit = 0.8;
   private final double outerLimit = 0.5;
   private final double innerLimit = 0.1;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final StepAdjustmentReachabilityConstraint reachabilityConstraint;
   private final FrameConvexPolygon2D reachabilityPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D inverseCaptureRegion = new FrameConvexPolygon2D();

   private final PoseReferenceFrame poseFrame = new PoseReferenceFrame("goalFrame", ReferenceFrame.getWorldFrame());

   private final YoFrameConvexPolygon2D yoInverseCaptureRegion = new YoFrameConvexPolygon2D("inverseCaptureRegion", ReferenceFrame.getWorldFrame(), 10, registry);

   public InverseCaptureRegionCalculator(WalkingControllerParameters walkingControllerParameters,
                                         ICPOptimizationParameters icpOptimizationParameters,
                                         SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                         YoRegistry parentRegistry,
                                         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      reachabilityConstraint = new StepAdjustmentReachabilityConstraint(soleZUpFrames,
                                                                               icpOptimizationParameters,
                                                                               walkingControllerParameters.getSteppingParameters(),
                                                                               "",
                                                                               false,
                                                                               registry,
                                                                               null);

      YoArtifactPolygon polygonArtifact = new YoArtifactPolygon("Inverse Capture Region", yoInverseCaptureRegion, Color.GREEN, false);
      yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D inversePoint = new FramePoint2D();
   private final FramePoint2D stepPosition = new FramePoint2D();

   public void reset()
   {
      yoInverseCaptureRegion.clear();
   }

   public void computeFromStepGoal(double minSwingAndTransferTime, SimpleFootstep stepGoal, double omega)
   {
      reachabilityConstraint.initializeReachabilityConstraint(stepGoal.getRobotSide(), stepGoal.getSoleFramePose());

      computeReachabilityPolygon(stepGoal);

      inverseCaptureRegion.clear();
//      inverseCaptureRegion.addVertices(reachabilityPolygon);
      stepPosition.set(stepGoal.getSoleFramePose().getPosition());

      double minExponential = Math.exp(-omega * 0.3);
      double maxExponential = Math.exp(-omega * 0.6);
      for (int i = 0; i < reachabilityPolygon.getNumberOfVertices(); i++)
      {
         inversePoint.interpolate(reachabilityPolygon.getVertex(i), stepPosition, minExponential);
         inverseCaptureRegion.addVertex(inversePoint);

         inversePoint.interpolate(reachabilityPolygon.getVertex(i), stepPosition, maxExponential);
         inverseCaptureRegion.addVertex(inversePoint);
      }

      inverseCaptureRegion.update();
      yoInverseCaptureRegion.set(inverseCaptureRegion);
   }

   private void computeReachabilityPolygon(SimpleFootstep stepGoal)
   {
      poseFrame.setPoseAndUpdate(stepGoal.getSoleFramePose());
      reachabilityPolygon.clear(poseFrame);

      // create an ellipsoid around the center of the forward and backward reachable limits
      double xRadius = 0.5 * (lengthLimit + lengthBackLimit);
      double yRadius = outerLimit - innerLimit;
      double centerX = lengthLimit - xRadius;
      double centerY = innerLimit;

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < nVertices; vertexIdx++)
      {
         double angle = Math.PI * vertexIdx / (nVertices - 1);
         double x = centerX + xRadius * Math.cos(angle);
         double y = centerY + yRadius * Math.sin(angle);
         reachabilityPolygon.addVertex(x, stepGoal.getRobotSide().negateIfLeftSide(y));
      }

      reachabilityPolygon.update();
      reachabilityPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
   }
}
