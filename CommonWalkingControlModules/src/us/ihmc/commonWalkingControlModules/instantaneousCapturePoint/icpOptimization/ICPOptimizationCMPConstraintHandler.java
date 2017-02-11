package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ICPOptimizationCMPConstraintHandler
{
   private final DoubleYoVariable maxCMPDoubleSupportExitForward;
   private final DoubleYoVariable maxCMPDoubleSupportExitSideways;
   private final DoubleYoVariable maxCMPSingleSupportExitForward;
   private final DoubleYoVariable maxCMPSingleSupportExitSideways;
   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationCMPConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry registry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      maxCMPDoubleSupportExitForward = new DoubleYoVariable("maxCMPDoubleSupportForwardExit", registry);
      maxCMPDoubleSupportExitSideways = new DoubleYoVariable("maxCMPDoubleSupportLateralExit", registry);

      maxCMPDoubleSupportExitForward.set(icpOptimizationParameters.getDoubleSupportMaxCMPForwardExit());
      maxCMPDoubleSupportExitSideways.set(icpOptimizationParameters.getDoubleSupportMaxCMPLateralExit());

      maxCMPSingleSupportExitForward = new DoubleYoVariable("maxCMPSingleSupportForwardExit", registry);
      maxCMPSingleSupportExitSideways = new DoubleYoVariable("maxCMPSingleSupportLateralExit", registry);

      maxCMPSingleSupportExitForward.set(icpOptimizationParameters.getSingleSupportMaxCMPForwardExit());
      maxCMPSingleSupportExitSideways.set(icpOptimizationParameters.getSingleSupportMaxCMPLateralExit());
   }

   private final FramePoint2d tempVertex = new FramePoint2d();
   public void updateCMPConstraintForDoubleSupport(ICPOptimizationSolver solver)
   {
      solver.resetSupportPolygonConstraint();

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide);

         for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
         {
            supportPolygon.getFrameVertex(i, tempVertex);
            solver.addSupportPolygonVertex(tempVertex, supportPolygon.getReferenceFrame(), maxCMPDoubleSupportExitForward.getDoubleValue(),
                  maxCMPDoubleSupportExitSideways.getDoubleValue());
         }
      }
   }

   public void updateCMPConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationSolver solver)
   {
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide);
      solver.resetSupportPolygonConstraint();

      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         supportPolygon.getFrameVertex(i, tempVertex);
         solver.addSupportPolygonVertex(tempVertex, supportPolygon.getReferenceFrame(), maxCMPSingleSupportExitForward.getDoubleValue(),
               maxCMPSingleSupportExitSideways.getDoubleValue());
      }
   }

}
