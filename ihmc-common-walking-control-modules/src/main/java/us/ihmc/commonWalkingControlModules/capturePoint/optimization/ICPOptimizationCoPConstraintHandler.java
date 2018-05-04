package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPOptimizationCoPConstraintHandler
{
   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   private final boolean hasICPControlPoygons;
   private final BooleanProvider useICPControlPolygons;
   private final YoBoolean keepCoPInsideSupportPolygon;

   public ICPOptimizationCoPConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPControlPolygons icpControlPolygons,
                                              BooleanProvider useICPControlPolygons, boolean hasICPControlPoygons, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.icpControlPolygons = icpControlPolygons;
      this.useICPControlPolygons = useICPControlPolygons;
      this.hasICPControlPoygons = hasICPControlPoygons;

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
            FrameConvexPolygon2D supportPolygon;
            // TODO: are icpControlPolygons != null and hasICPControlPoygons redundant?
            if (useICPControlPolygons.getValue() && icpControlPolygons != null && hasICPControlPoygons)
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
         FrameConvexPolygon2D supportPolygon;
         // TODO: are icpControlPolygons != null and hasICPControlPoygons redundant?
         if (useICPControlPolygons.getValue() && icpControlPolygons != null && hasICPControlPoygons)
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
