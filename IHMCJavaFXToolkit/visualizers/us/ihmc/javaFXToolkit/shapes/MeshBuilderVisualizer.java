package us.ihmc.javaFXToolkit.shapes;

import java.util.Random;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotics.random.RandomTools;

public class MeshBuilderVisualizer extends Application
{
   private enum MeshToDisplay {BOX, LINE, CYLINDER, CONE}
   private static final MeshToDisplay MESH_TO_DISPLAY = MeshToDisplay.CONE;

   public MeshBuilderVisualizer()
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      switch (MESH_TO_DISPLAY)
      {
      case BOX:
         addRandomBoxes(meshBuilder);
         break;
      case LINE:
         addLines(meshBuilder);
         break;
      case CYLINDER:
         addCylinders(meshBuilder);
         break;
      case CONE:
         addCones(meshBuilder);
         break;
      default:
         break;
      }

//      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cone(0.3, 0.1, 64)));
//      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.GenTruncatedCone(0.3, 0.1, 0.1, 0.1, 0.1, 64)));
//      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.GenTruncatedCone(0.3, 0.1, 0.1, 0.1, 0.1, 64)));
//      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cylinder(0.1, 0.3, 64)));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(Color.CYAN);
      material.setSpecularColor(Color.CYAN.brighter());
      meshView.setMaterial(material);
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public void addRandomBoxes(JavaFXMeshBuilder meshBuilder)
   {
      int count = 0;
      for (float x = -5.0f; x <= 5.0f; x += 0.055f)
      {
         for (float y = -2.0f; y <= 2.0f; y += 0.055f)
         {
            meshBuilder.addBox(0.05f, 0.05f, 0.05f, new Vector3D32(x, y, RandomTools.generateRandomFloatInRange(new Random(), -2.0f, 2.0f)));
            count++;
         }
      }
      System.out.println("Number of boxes: " + count);
   }

   private void addLines(JavaFXMeshBuilder meshBuilder)
   {
      Point3D start = new Point3D(0.3, 0.0, -0.);
      Point3D end = new Point3D(0.0, 0.3, 0.0);
      double lineWidth = 0.01;
      meshBuilder.addLine(start, end, lineWidth);
   }

   private void addCylinders(JavaFXMeshBuilder meshBuilder)
   {
      Point3D cylinderPosition = new Point3D(1.0, 0.0, 0.0);
      double height = 0.3;
      double radius = 0.1;
      meshBuilder.addCylinder(height, radius, cylinderPosition);
//      meshBuilder.addMesh(MeshDataGenerator.ArcTorus(0.0, 2.0 * Math.PI, 0.3, 0.01, 128));
      meshBuilder.addMesh(MeshDataGenerator.Cylinder(radius, height, 64));
   }

   private void addCones(JavaFXMeshBuilder meshBuilder)
   {
      Point3D conePosition = new Point3D(0.4, 0.0, 0.0);
      double height = 0.3;
      double radius = 0.1;
      meshBuilder.addCone(height, radius, conePosition);
//      meshBuilder.addMesh(MeshDataGenerator.ArcTorus(0.0, 2.0 * Math.PI, 0.3, 0.01, 128));
      meshBuilder.addMesh(MeshDataGenerator.Cone(height, radius, 64));
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
