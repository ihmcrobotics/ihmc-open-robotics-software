package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.List;

public class StaticEquilibriumSolverVisualizer
{
   private static final boolean showSupportRegion = true;

   public StaticEquilibriumSolverVisualizer(StaticEquilibriumSolverInput input)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));

      StaticEquilibriumSolver solver = new StaticEquilibriumSolver();
      scs.getRootRegistry().addChild(solver.getRegistry());
      scs.addYoGraphicsListRegistry(solver.getGraphicsListRegistry());
      solver.setTickAndUpdatable(scs);
      solver.initialize(input);
      solver.solve();

      System.out.println("----- support region ----");
      ConvexPolygon2D supportRegion0 = solver.getSupportRegion();
      for (int i = 0; i < supportRegion0.getNumberOfVertices(); i++)
      {
         System.out.println(supportRegion0.getVertex(i));
      }

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

      if (showSupportRegion)
      {
         ConvexPolygon2D supportRegion = solver.getSupportRegion();

         double renderedHeight = 0.1;
         supportRegionGraphics.identity();
         supportRegionGraphics.translate(0.0, 0.0, renderedHeight);
         supportRegionGraphics.addExtrudedPolygon(supportRegion, 0.01, YoAppearance.Glass());
      }

      scs.addStaticLinkGraphics(supportRegionGraphics);
      scs.setGroundVisible(false);
      scs.startOnAThread();
      scs.cropBuffer();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleFlatGround();
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly();
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot();
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyOut();
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyIn();
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createFlatSquare();
      // StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeet();
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeetWithHandhold();

      new StaticEquilibriumSolverVisualizer(input);
   }
}
