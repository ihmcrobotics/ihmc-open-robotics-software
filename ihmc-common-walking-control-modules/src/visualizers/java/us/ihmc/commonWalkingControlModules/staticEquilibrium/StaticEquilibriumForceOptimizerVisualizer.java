package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
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

      StaticEquilibriumSolver regionSolver = new StaticEquilibriumSolver();
      regionSolver.initialize(input);
      regionSolver.solve();
      ConvexPolygon2D supportRegion = new ConvexPolygon2D();
      regionSolver.getSupportRegion().forEach(supportRegion::addVertex);
      supportRegion.update();

      double renderedHeight = 0.1;
      Graphics3DObject supportRegionGraphics = new Graphics3DObject();
      supportRegionGraphics.identity();
      supportRegionGraphics.translate(0.0, 0.0, renderedHeight);
      supportRegionGraphics.addExtrudedPolygon(supportRegion, 0.01, YoAppearance.Glass());
      scs.addStaticLinkGraphics(supportRegionGraphics);

      StaticEquilibriumForceOptimizer pointVerifier = new StaticEquilibriumForceOptimizer();

      YoFramePoint3D comPosition = new YoFramePoint3D("comPosition", ReferenceFrame.getWorldFrame(), scs.getRootRegistry());
      comPosition.setZ(renderedHeight);
      YoGraphicPosition comGraphic = new YoGraphicPosition("comPositionGraphic", comPosition, 0.03, YoAppearance.Red());
      pointVerifier.getGraphicsListRegistry().registerYoGraphic("comGraphic", comGraphic);

      scs.getRootRegistry().addChild(pointVerifier.getRegistry());
      scs.addYoGraphicsListRegistry(pointVerifier.getGraphicsListRegistry());

      Runnable solverRunnable = () ->
      {
         pointVerifier.solve(input, new Point2D(comPosition.getX(), comPosition.getY()));
         System.out.println("succeeded: " + pointVerifier.feasibleSolutionFound());
         scs.tickAndUpdate();
      };

      comPosition.getYoX().addListener(v -> solverRunnable.run());
      comPosition.getYoY().addListener(v -> solverRunnable.run());

      Graphics3DObject groundGraphics = new Graphics3DObject();

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         FramePoint3D contactPoint = input.getContactPointPositions().get(i);

         groundGraphics.identity();
         groundGraphics.translate(contactPoint);
         groundGraphics.addSphere(0.03, YoAppearance.Black());

         Quaternion surfaceOrientation = new Quaternion();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, input.getSurfaceNormals().get(i), surfaceOrientation);
         groundGraphics.rotate(surfaceOrientation);
         groundGraphics.translate(0.0, 0.0, -0.01);
         groundGraphics.addCylinder(0.02, 0.15, YoAppearance.Beige());
      }

      scs.addStaticLinkGraphics(groundGraphics);
      scs.setGroundVisible(false);
      scs.startOnAThread();
      scs.cropBuffer();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleFlatGround();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyOut();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyIn();

      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createFlatSquare();

      //      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeet();
//      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeet();

      new StaticEquilibriumForceOptimizerVisualizer(input);
   }
}
