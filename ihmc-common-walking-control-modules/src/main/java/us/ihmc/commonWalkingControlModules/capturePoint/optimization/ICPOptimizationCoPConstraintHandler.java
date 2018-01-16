package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPOptimizationCoPConstraintHandler
{
   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   private final YoBoolean useICPControlPolygons;
   private final YoBoolean keepCoPInsideSupportPolygon;

   public ICPOptimizationCoPConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                              YoBoolean useICPControlPolygons, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.icpControlPolygons = icpControlPolygons;
      this.useICPControlPolygons = useICPControlPolygons;

      keepCoPInsideSupportPolygon = new YoBoolean("keepCoPInsideSupportPolygon", parentRegistry);
      keepCoPInsideSupportPolygon.set(true);
   }

   public void updateCoPConstraintForDoubleSupport(ICPOptimizationQPSolver solver)
   {
      solver.resetCoPLocationConstraint();

      if (keepCoPInsideSupportPolygon.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FrameConvexPolygon2d supportPolygon;
            if (useICPControlPolygons.getBooleanValue() && icpControlPolygons != null)
               supportPolygon = icpControlPolygons.getFootControlPolygonInWorldFrame(robotSide);
            else
               supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(robotSide);
            solver.addSupportPolygon(supportPolygon);
         }
      }
   }

   public void updateCoPConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationQPSolver solver)
   {
      solver.resetCoPLocationConstraint();

      if (keepCoPInsideSupportPolygon.getBooleanValue())
      {
         FrameConvexPolygon2d supportPolygon;
         if (useICPControlPolygons.getBooleanValue() && icpControlPolygons != null)
            supportPolygon = icpControlPolygons.getFootControlPolygonInWorldFrame(supportSide);
         else
            supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(supportSide);
         solver.addSupportPolygon(supportPolygon);
      }
   }

   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.keepCoPInsideSupportPolygon.set(keepCoPInsideSupportPolygon);
   }
}
