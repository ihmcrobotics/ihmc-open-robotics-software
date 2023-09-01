package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;

import javax.swing.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class MultiContactFrictionBasedSupportRegionSolverVisualizer
{
   private static final boolean showSupportRegion = true;
   private static final double visualizationScale = 2e-4;

   public MultiContactFrictionBasedSupportRegionSolverVisualizer(MultiContactFrictionBasedSupportRegionSolverInput input)
   {
      runWithSCS1(input);
//      runWithSCS2(input);
   }

   private void runWithSCS1(MultiContactFrictionBasedSupportRegionSolverInput input)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));

      MultiContactFrictionBasedSupportRegionSolver solver = new MultiContactFrictionBasedSupportRegionSolver();
      scs.getRootRegistry().addChild(solver.getRegistry());
      scs.addYoGraphicsListRegistry(solver.getGraphicsListRegistry());

      solver.setTickAndUpdatable(scs);
      solver.initialize(input);
      solver.solve();

      LogTools.info("----- Support Region -----");
      ConvexPolygon2DReadOnly supportRegion0 = solver.getSupportRegion();
      for (int i = 0; i < supportRegion0.getNumberOfVertices(); i++)
      {
         System.out.println("\t" + supportRegion0.getVertex(i));
      }

      Graphics3DObject supportRegionGraphics = new Graphics3DObject();

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         FramePoint3D contactPoint = input.getContactPointPositions().get(i);

         supportRegionGraphics.identity();
         supportRegionGraphics.translate(contactPoint);
         supportRegionGraphics.addSphere(0.01, YoAppearance.Black());

         Quaternion surfaceOrientation = new Quaternion();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, input.getSurfaceNormals().get(i), surfaceOrientation);
         supportRegionGraphics.rotate(surfaceOrientation);
         supportRegionGraphics.translate(0.0, 0.0, -0.005);
         supportRegionGraphics.addCylinder(0.005, 0.04, YoAppearance.Beige());

         ContactPointActuationConstraint actuationConstraint = input.getActuationConstraints().get(i);
         if (!actuationConstraint.isMaxNormalForceConstraint())
         {
            supportRegionGraphics.identity();
            supportRegionGraphics.translate(contactPoint);
            supportRegionGraphics.addMeshData(newConvexPolytope3DMesh(actuationConstraint.getPolytopeConstraint()), YoAppearance.Glass());
         }
      }

      if (showSupportRegion)
      {
         double renderedHeight = solver.getAverageContactPointPosition().getZ();

         ConvexPolygon2DReadOnly supportRegion = solver.getSupportRegion();
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

   private void runWithSCS2(MultiContactFrictionBasedSupportRegionSolverInput input)
   {
      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2();

      MultiContactFrictionBasedSupportRegionSolver solver = new MultiContactFrictionBasedSupportRegionSolver();
      scs2.getRootRegistry().addChild(solver.getRegistry());
      //      scs2.addYoGraphicsListRegistry(solver.getGraphicsListRegistry());

      solver.setTickAndUpdatable(new TickAndUpdatable()
      {
         @Override
         public void tickAndUpdate()
         {
            scs2.simulate(1);
         }

         @Override
         public void tickAndUpdate(double timeToSetInSeconds)
         {
            scs2.simulate(timeToSetInSeconds);
         }
      });
      solver.initialize(input);
      solver.solve();

      LogTools.info("----- Support Region -----");
      ConvexPolygon2DReadOnly supportRegion0 = solver.getSupportRegion();
      for (int i = 0; i < supportRegion0.getNumberOfVertices(); i++)
      {
         System.out.println("\t" + supportRegion0.getVertex(i));
      }

      VisualDefinitionFactory supportRegionGraphics = new VisualDefinitionFactory();

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         FramePoint3D contactPoint = input.getContactPointPositions().get(i);

         supportRegionGraphics.identity();
         supportRegionGraphics.appendTranslation(contactPoint);
         supportRegionGraphics.addSphere(0.03, new ColorDefinition());

         Quaternion surfaceOrientation = new Quaternion();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, input.getSurfaceNormals().get(i), surfaceOrientation);
         supportRegionGraphics.appendRotation(surfaceOrientation);
         supportRegionGraphics.appendTranslation(0.0, 0.0, -0.01);

         ColorDefinition beige = new ColorDefinition();
         beige.setRed(255);
         beige.setGreen(255);
         beige.setBlue(180);
         supportRegionGraphics.addCylinder(0.02, 0.15, beige);
      }

      if (showSupportRegion)
      {
         double renderedHeight = solver.getAverageContactPointPosition().getZ();

         ConvexPolygon2DReadOnly supportRegion = solver.getSupportRegion();
         supportRegionGraphics.identity();
         supportRegionGraphics.appendTranslation(0.0, 0.0, renderedHeight);

         ColorDefinition glass = new ColorDefinition();
         glass.setRed(120);
         glass.setGreen(180);
         glass.setBlue(240);
         glass.setAlpha(0.5);
         supportRegionGraphics.addExtrudedPolygon(supportRegion, 0.01, glass);
      }

      supportRegionGraphics.identity();

      scs2.addStaticVisuals(supportRegionGraphics.getVisualDefinitions());
      scs2.startSimulationThread();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args) throws Exception
   {
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createTriangleFlatGround();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createTriangleTiltedOutSlightly();
      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createTriangleTiltedOutALot();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createTriangleOneTiltedFullyOut();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createTriangleOneTiltedFullyIn();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createFlatSquare();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createBipedFeet();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createBipedFeet(1.0, -1.0, 0.6);
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createBipedFeetWithSingleHandhold();
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactSupportRegionSolverInputExamples.createBipedFeetWithTwoHandholds();

//      JFileChooser fileChooser = new JFileChooser();
//      File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);
//
//      fileChooser.setCurrentDirectory(logDirectory);
//      int chooserState = fileChooser.showOpenDialog(null);
//
//      if (chooserState != JFileChooser.APPROVE_OPTION)
//         return;
//
//      File file = fileChooser.getSelectedFile();
//
//      BufferedReader dataFileReader = new BufferedReader(new FileReader(file));
//      MultiContactFrictionBasedSupportRegionSolverInput input = MultiContactFrictionBasedSupportRegionSolverInput.loadFromFile(new BufferedReader(dataFileReader));

      new MultiContactFrictionBasedSupportRegionSolverVisualizer(input);
   }

   private static MeshDataHolder newConvexPolytope3DMesh(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      ConvexPolytope3D scaledPolytope = new ConvexPolytope3D();
      for (int i = 0; i < convexPolytope3D.getNumberOfVertices(); i++)
      {
         Vertex3D scaledVertex = new Vertex3D(convexPolytope3D.getVertex(i));
         scaledVertex.scale(ContactPoint.forceVectorGraphicScale);
         scaledPolytope.addVertex(scaledVertex);
      }

      MeshDataBuilder meshBuilder = new MeshDataBuilder();

      for (Face3DReadOnly face : scaledPolytope.getFaces())
      {
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         meshBuilder.addMesh(MeshDataGenerator.Polygon(ccwFaceVertices));
      }

      return meshBuilder.generateMeshDataHolder();
   }
}
