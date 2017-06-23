package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class ICPOptimizationReachabilityConstraintHandler
{
   private final SideDependentList<YoFrameConvexPolygon2d> reachabilityPolygons = new SideDependentList<>();

   public ICPOptimizationReachabilityConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters,
         String yoNamePrefix, YoVariableRegistry registry)
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
   }

   public void updateReachabilityConstraintForSingleSupport(RobotSide supportSide, ICPQPOptimizationSolver solver)
   {
      solver.resetReachabilityConstraint();

      FrameConvexPolygon2d reachabilityPolygon = reachabilityPolygons.get(supportSide).getFrameConvexPolygon2d();
      reachabilityPolygon.changeFrame(ReferenceFrame.getWorldFrame());
      solver.addReachabilityPolygon(reachabilityPolygon);
   }

   public void updateReachabilityConstraintForDoubleSupport(ICPQPOptimizationSolver solver)
   {
      solver.resetReachabilityConstraint();
   }
}
