package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ICPOptimizationCoPConstraintHandler
{
   private final DoubleYoVariable maxCoPDoubleSupportExitForward;
   private final DoubleYoVariable maxCoPDoubleSupportExitSideways;
   private final DoubleYoVariable maxCoPSingleSupportExitForward;
   private final DoubleYoVariable maxCoPSingleSupportExitSideways;
   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationCoPConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters,
         String yoNamePrefix, YoVariableRegistry registry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      maxCoPDoubleSupportExitForward = new DoubleYoVariable(yoNamePrefix + "MaxCoPDoubleSupportForwardExit", registry);
      maxCoPDoubleSupportExitSideways = new DoubleYoVariable(yoNamePrefix + "MaxCoPDoubleSupportLateralExit", registry);

      maxCoPDoubleSupportExitForward.set(icpOptimizationParameters.getDoubleSupportMaxCoPForwardExit());
      maxCoPDoubleSupportExitSideways.set(icpOptimizationParameters.getDoubleSupportMaxCoPLateralExit());

      maxCoPSingleSupportExitForward = new DoubleYoVariable(yoNamePrefix + "MaxCoPSingleSupportForwardExit", registry);
      maxCoPSingleSupportExitSideways = new DoubleYoVariable(yoNamePrefix + "MaxCoPSingleSupportLateralExit", registry);

      maxCoPSingleSupportExitForward.set(icpOptimizationParameters.getSingleSupportMaxCoPForwardExit());
      maxCoPSingleSupportExitSideways.set(icpOptimizationParameters.getSingleSupportMaxCoPLateralExit());
   }

   private final FramePoint2d tempVertex = new FramePoint2d();
   public void updateCoPConstraintForDoubleSupport(ICPOptimizationSolver solver)
   {
      solver.resetSupportPolygonConstraint();

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide);

         for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
         {
            supportPolygon.getFrameVertex(i, tempVertex);
            solver.addSupportPolygonVertex(tempVertex, supportPolygon.getReferenceFrame(), maxCoPDoubleSupportExitForward.getDoubleValue(),
                  maxCoPDoubleSupportExitSideways.getDoubleValue());
         }
      }
   }

   public void updateCoPConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationSolver solver)
   {
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide);
      solver.resetSupportPolygonConstraint();

      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         supportPolygon.getFrameVertex(i, tempVertex);
         solver.addSupportPolygonVertex(tempVertex, supportPolygon.getReferenceFrame(), maxCoPSingleSupportExitForward.getDoubleValue(),
               maxCoPSingleSupportExitSideways.getDoubleValue());
      }
   }

}
