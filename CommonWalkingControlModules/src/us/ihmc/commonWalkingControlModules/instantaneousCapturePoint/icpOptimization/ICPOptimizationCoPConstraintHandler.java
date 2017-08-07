package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ICPOptimizationCoPConstraintHandler
{
   private static final boolean useControlPolygons = true;

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   public ICPOptimizationCoPConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.icpControlPolygons = icpControlPolygons;
   }

   public void updateCoPConstraintForDoubleSupport(ICPQPOptimizationSolver solver)
   {
      solver.resetCoPLocationConstraint();

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon;
         if (useControlPolygons && icpControlPolygons != null)
            supportPolygon = icpControlPolygons.getFootControlPolygonInWorldFrame(robotSide);
         else
            supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide);
         solver.addSupportPolygon(supportPolygon);
      }
   }

   public void updateCoPConstraintForDoubleSupport(SimpleICPOptimizationQPSolver solver)
   {
      solver.resetCoPLocationConstraint();

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon;
         if (useControlPolygons && icpControlPolygons != null)
            supportPolygon = icpControlPolygons.getFootControlPolygonInWorldFrame(robotSide);
         else
            supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide);
         solver.addSupportPolygon(supportPolygon);
      }
   }

   public void updateCoPConstraintForSingleSupport(RobotSide supportSide, ICPQPOptimizationSolver solver)
   {
      solver.resetCoPLocationConstraint();

      FrameConvexPolygon2d supportPolygon;
      if (useControlPolygons && icpControlPolygons != null)
         supportPolygon = icpControlPolygons.getFootControlPolygonInWorldFrame(supportSide);
      else
         supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide);
      solver.addSupportPolygon(supportPolygon);
   }

   public void updateCoPConstraintForSingleSupport(RobotSide supportSide, SimpleICPOptimizationQPSolver solver)
   {
      solver.resetCoPLocationConstraint();

      FrameConvexPolygon2d supportPolygon;
      if (useControlPolygons && icpControlPolygons != null)
         supportPolygon = icpControlPolygons.getFootControlPolygonInWorldFrame(supportSide);
      else
         supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide);
      solver.addSupportPolygon(supportPolygon);
   }
}
