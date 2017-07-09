package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ICPOptimizationCoPConstraintHandler
{
   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationCoPConstraintHandler(BipedSupportPolygons bipedSupportPolygons)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
   }

   public void updateCoPConstraintForDoubleSupport(ICPQPOptimizationSolver solver)
   {
      solver.resetCoPLocationConstraint();

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide);
         solver.addSupportPolygon(supportPolygon);
      }
   }

   public void updateCoPConstraintForSingleSupport(RobotSide supportSide, ICPQPOptimizationSolver solver)
   {
      solver.resetCoPLocationConstraint();

      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide);
      solver.addSupportPolygon(supportPolygon);
   }
}
