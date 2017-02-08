package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;

public class ICPOptimizationReachabilityConstraintHandler
{
   private final DoubleYoVariable lateralReachabilityOuterLimit;
   private final DoubleYoVariable lateralReachabilityInnerLimit;
   private final DoubleYoVariable forwardReachabilityLimit;
   private final DoubleYoVariable backwardReachabilityLimit;

   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationReachabilityConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry registry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      lateralReachabilityOuterLimit = new DoubleYoVariable("lateralReachabilityOuterLimit", registry);
      lateralReachabilityInnerLimit = new DoubleYoVariable("lateralReachabilityInnerLimit", registry);

      forwardReachabilityLimit = new DoubleYoVariable("forwardReachabilityLimit", registry);
      backwardReachabilityLimit = new DoubleYoVariable("backwardReachabilityLimit", registry);

      lateralReachabilityInnerLimit.set(icpOptimizationParameters.getLateralReachabilityInnerLimit());
      lateralReachabilityOuterLimit.set(Math.max(lateralReachabilityInnerLimit.getDoubleValue(), icpOptimizationParameters.getLateralReachabilityOuterLimit()));

      forwardReachabilityLimit.set(icpOptimizationParameters.getForwardReachabilityLimit());
      backwardReachabilityLimit.set(icpOptimizationParameters.getBackwardReachabilityLimit());
   }

   private final FramePoint2d tempVertex = new FramePoint2d();
   public void updateReachabilityConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationSolver solver)
   {
      solver.resetReachabilityConstraint();

      double lateralInnerLimit = lateralReachabilityInnerLimit.getDoubleValue();
      double lateralOuterLimit = lateralReachabilityOuterLimit.getDoubleValue();

      lateralInnerLimit = supportSide.negateIfLeftSide(lateralInnerLimit);
      lateralOuterLimit = supportSide.negateIfLeftSide(lateralOuterLimit);

      ReferenceFrame supportSoleFrame = bipedSupportPolygons.getSoleZUpFrames().get(supportSide);

      tempVertex.setToZero(supportSoleFrame);
      tempVertex.set(forwardReachabilityLimit.getDoubleValue(), lateralInnerLimit);
      solver.addReachabilityVertex(tempVertex, supportSoleFrame);

      tempVertex.setToZero(supportSoleFrame);
      tempVertex.set(forwardReachabilityLimit.getDoubleValue(), lateralOuterLimit);
      solver.addReachabilityVertex(tempVertex, supportSoleFrame);

      tempVertex.setToZero(supportSoleFrame);
      tempVertex.set(backwardReachabilityLimit.getDoubleValue(), lateralInnerLimit);
      solver.addReachabilityVertex(tempVertex, supportSoleFrame);

      tempVertex.setToZero(supportSoleFrame);
      tempVertex.set(backwardReachabilityLimit.getDoubleValue(), lateralOuterLimit);
      solver.addReachabilityVertex(tempVertex, supportSoleFrame);
   }

   public void updateReachabilityConstraintForDoubleSupport(ICPOptimizationSolver solver)
   {
      solver.resetReachabilityConstraint();
   }
}
