package us.ihmc.javaFXToolkit.shapes;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class SegmentedLine3DMeshDataGeneratorVisualizer extends Application
{

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.4);

      long startTime = System.nanoTime();
      MeshDataHolder[] meshDataHolders = createMesh();
      long endTime = System.nanoTime();
      System.out.println("Took: " + Conversions.nanosecondsToSeconds(endTime - startTime) + " sec");

      for (int i = 0; i < meshDataHolders.length; i++)
      {
         double hue = i / (meshDataHolders.length - 1.0) * 360.0;
         MeshDataHolder meshDataHolder = meshDataHolders[i];
         Material defaultMaterial = new PhongMaterial(Color.hsb(hue, 0.9, 0.9));
         MeshView meshView = new MeshView();
         meshView.setMesh(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
         meshView.setMaterial(defaultMaterial);
         view3dFactory.addNodeToView(meshView);
      }

      primaryStage.setMaximized(true);
      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private MeshDataHolder[] createMesh()
   {
      int numberOfSections = 1000;
      SegmentedLine3DMeshDataGenerator deformablePipeMeshCalculator = new SegmentedLine3DMeshDataGenerator(numberOfSections, 64, 0.02);
      Point3D[] centers = new Point3D[numberOfSections];

      double lx = 1.0;
      double phase = Math.PI / 2.0;

      for (int i = 0; i < numberOfSections; i++)
      {
         double x = i * lx / (numberOfSections - 1.0);
         double y = 0.3 * Math.cos(2.0 * Math.PI * 4.0 * x + phase);
         double z = 0.1 * Math.sin(2.0 * Math.PI * 4.0 * x + phase);
         centers[i] = new Point3D(x, y, z);
      }
      deformablePipeMeshCalculator.compute(centers);
      return deformablePipeMeshCalculator.getMeshDataHolders();
   }

   public static void main(String[] args)
   {
      launch();
   }
}
