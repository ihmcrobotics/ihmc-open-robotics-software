package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ICPOptimizationCoPConstraintHandler
{
   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationCoPConstraintHandler(BipedSupportPolygons bipedSupportPolygons)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
   }

   public void updateCoPConstraintForDoubleSupport(ICPOptimizationSolver solver)
   {
      solver.resetCoPLocationConstraint();

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide);
         solver.addSupportPolygon(supportPolygon);
      }
   }

   public void updateCoPConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationSolver solver)
   {
      solver.resetCoPLocationConstraint();

      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide);
      solver.addSupportPolygon(supportPolygon);
   }
}
