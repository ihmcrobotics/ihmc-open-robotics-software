package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ICPOptimizationCMPConstraintHandler
{
   private final DoubleYoVariable maxCMPExitForward;
   private final DoubleYoVariable maxCMPExitSideways;
   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationCMPConstraintHandler(BipedSupportPolygons bipedSupportPolygons, ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry registry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      maxCMPExitForward = new DoubleYoVariable("maxCMPForwardExit", registry);
      maxCMPExitSideways = new DoubleYoVariable("maxCMPLateralExit", registry);

      maxCMPExitForward.set(icpOptimizationParameters.getMaxCMPForwardExit());
      maxCMPExitSideways.set(icpOptimizationParameters.getMaxCMPLateralExit());
   }

   private final FramePoint2d tempVertex = new FramePoint2d();
   public void initializeCMPConstraintForDoubleSupport(ICPOptimizationSolver solver)
   {
      int numberOfVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         numberOfVertices += bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide).getNumberOfVertices();
      solver.setNumberOfVertices(numberOfVertices);

      numberOfVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide);

         for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
         {
            supportPolygon.getFrameVertex(i, tempVertex);
            solver.setSupportPolygonVertex(numberOfVertices + i, tempVertex, supportPolygon.getReferenceFrame(), maxCMPExitForward.getDoubleValue(),
                  maxCMPExitSideways.getDoubleValue());
         }

         numberOfVertices += supportPolygon.getNumberOfVertices();
      }
   }

   public void initializeCMPConstraintForSingleSupport(RobotSide supportSide, ICPOptimizationSolver solver)
   {
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide);
      solver.setNumberOfVertices(supportPolygon.getNumberOfVertices());
      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         supportPolygon.getFrameVertex(i, tempVertex);
         solver.setSupportPolygonVertex(i, tempVertex, supportPolygon.getReferenceFrame(), maxCMPExitForward.getDoubleValue(),
               maxCMPExitSideways.getDoubleValue());
      }
   }

}
