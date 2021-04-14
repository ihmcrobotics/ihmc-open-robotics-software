package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;

public class StaticEquilibriumForceOptimizerVisualizer
{
   public StaticEquilibriumForceOptimizerVisualizer(StaticEquilibriumSolverInput input)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));

      StaticEquilibriumForceOptimizer solver = new StaticEquilibriumForceOptimizer();

      YoFramePoint3D comPosition = new YoFramePoint3D("comPosition", ReferenceFrame.getWorldFrame(), scs.getRootRegistry());
      YoGraphicPosition comGraphic = new YoGraphicPosition("comPositionGraphic", comPosition, 0.03, YoAppearance.Red());
      solver.getGraphicsListRegistry().registerYoGraphic("comGraphic", comGraphic);

      scs.getRootRegistry().addChild(solver.getRegistry());
      scs.addYoGraphicsListRegistry(solver.getGraphicsListRegistry());

      Runnable solverRunnable = () ->
      {
         solver.solve(input, new Point2D(comPosition.getX(), comPosition.getY()));
         System.out.println("succeeded: " + solver.feasibleSolutionFound());
         scs.tickAndUpdate();
      };

      comPosition.getYoX().addListener(v -> solverRunnable.run());
      comPosition.getYoY().addListener(v -> solverRunnable.run());

      Graphics3DObject supportRegionGraphics = new Graphics3DObject();

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         FramePoint3D contactPoint = input.getContactPointPositions().get(i);

         supportRegionGraphics.identity();
         supportRegionGraphics.translate(contactPoint);
         supportRegionGraphics.addSphere(0.03, YoAppearance.Black());

         Quaternion surfaceOrientation = new Quaternion();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, input.getSurfaceNormals().get(i), surfaceOrientation);
         supportRegionGraphics.rotate(surfaceOrientation);
         supportRegionGraphics.translate(0.0, 0.0, -0.01);
         supportRegionGraphics.addCylinder(0.02, 0.15, YoAppearance.Beige());
      }

      scs.addStaticLinkGraphics(supportRegionGraphics);
      scs.setGroundVisible(false);
      scs.startOnAThread();
      scs.cropBuffer();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleFlatGround();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyOut();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyIn();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeet();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeet();

      new StaticEquilibriumForceOptimizerVisualizer(input);
   }
}
