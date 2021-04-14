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
      solver.solve(input);

//      List<Point2D> supportRegion0 = solver.getSupportRegion();
//      for (int i = 0; i < supportRegion0.size(); i++)
//      {
//         System.out.println(supportRegion0.get(i));
//      }

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
         supportRegionGraphics.addCylinder(0.02, 0.3, YoAppearance.Beige());
      }

      if (showSupportRegion)
      {
         ConvexPolygon2D supportRegion = new ConvexPolygon2D();
         solver.getSupportRegion().forEach(supportRegion::addVertex);
         supportRegion.update();

         double renderedHeight = 0.1;
         supportRegionGraphics.identity();
         supportRegionGraphics.translate(0.0, 0.0, renderedHeight);
         supportRegionGraphics.addExtrudedPolygon(supportRegion, 0.01, YoAppearance.Glass());
      }

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(supportRegionGraphics);
      scs.startOnAThread();
      scs.cropBuffer();

      ThreadTools.sleepForever();
   }

   private static StaticEquilibriumSolverInput createInput(double... angles)
   {
      StaticEquilibriumSolverInput input = new StaticEquilibriumSolverInput();

      double distance = 1.0;
      for (int i = 0; i < 3; i++)
      {
         double theta = i * 2.0 * Math.PI / 3.0;
         FramePoint3D contactPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), distance * Math.cos(theta), distance * Math.sin(theta), 0.0);

         Vector3D axis = new Vector3D(contactPoint);
         axis.normalize();
         axis.set(-axis.getY(), axis.getX(), 0.0);

         AxisAngle axisAngle = new AxisAngle(axis, angles[i]);
         FrameVector3D normal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         axisAngle.transform(normal);

         input.addContactPoint(contactPoint, normal);
      }

      input.setRobotMass(1.0);

      return input;
   }

   public static void main(String[] args)
   {
//      // flat ground
//      double theta0 = 0.0;
//      double theta1 = 0.0;
//      double theta2 = 0.0;

//       tilted out a little
      double theta0 = Math.toRadians(30.0);
      double theta1 = Math.toRadians(30.0);
      double theta2 = Math.toRadians(30.0);

      //      // tilted out a lot
//      double theta0 = Math.toRadians(60.0);
//      double theta1 = Math.toRadians(60.0);
//      double theta2 = Math.toRadians(60.0);

//      // 2 flat one perpendicular out
//      double theta0 = Math.toRadians(-90.0);
//      double theta1 = 0.0;
//      double theta2 = 0.0;

//      // 2 flat one perpendicular in
//      double theta0 = Math.toRadians(-90.0);
//      double theta2 = 0.0;
//      double theta1 = 0.0;

      double height0 = 0.0;
      double height1 = 0.0;
      double height2 = 0.0;

      StaticEquilibriumSolverInput input = createInput(theta0, theta1, theta2);

      input.getContactPointPositions().get(0).setZ(height0);
      input.getContactPointPositions().get(1).setZ(height1);
      input.getContactPointPositions().get(2).setZ(height2);

      new StaticEquilibriumSolverVisualizer(input);
   }
}
