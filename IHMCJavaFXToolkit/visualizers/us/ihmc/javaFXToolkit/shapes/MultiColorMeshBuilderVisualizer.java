package us.ihmc.javaFXToolkit.shapes;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class MultiColorMeshBuilderVisualizer extends Application
{
   private enum MeshToDisplay {BOX, LINE, MULTI_LINE}
   private static final MeshToDisplay MESH_TO_DISPLAY = MeshToDisplay.BOX;

   private Random random = new Random(23423L);

   public MultiColorMeshBuilderVisualizer()
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

      Color[] colors = {Color.RED, Color.YELLOW, Color.BEIGE, Color.CHOCOLATE, Color.ANTIQUEWHITE, Color.CYAN};

//      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
//      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorPalette2D());

      switch (MESH_TO_DISPLAY)
      {
      case BOX:
         view3dFactory.addNodesToView(addRandomBoxes(colors, meshBuilder));
         break;
      case LINE:
         addLine(meshBuilder);
      case MULTI_LINE:
         addMultiLine(meshBuilder);
      default:
         break;
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private void addMultiLine(JavaFXMultiColorMeshBuilder meshBuilder)
   {
      List<Point3D> points = new ArrayList<>();
      double radius = 0.4;
      Random random = new Random();

      for (double angle = 0.0; angle < 2.0 * Math.PI; angle += 2.0 * Math.PI / 50.0)
      {
         double x = radius * random.nextDouble() * Math.cos(angle);
         double y = radius * Math.sin(angle);
         double z = 0.1 * random.nextDouble();
         points.add(new Point3D(x, y, z));
      }
      meshBuilder.addMultiLine(points, 0.01, Color.YELLOWGREEN, true);
   }

   private void addLine(JavaFXMultiColorMeshBuilder meshBuilder)
   {
      Point3D start = new Point3D(0.3, 0.0, -0.);
      Point3D end = new Point3D(0.0, 0.3, 0.0);
      double lineWidth = 0.01;
      Color color = Color.RED;
      meshBuilder.addLine(start, end, lineWidth, color);
   }

   public List<Box> addRandomBoxes(Color[] colors, JavaFXMultiColorMeshBuilder meshBuilder)
   {
      int count = 0;
      Random random = new Random();
      List<Box> boxes = new ArrayList<>();

      for (float x = -1.0f; x <= 1.0f; x += 0.055f)
      {
         for (float y = -1.0f; y <= 1.0f; y += 0.055f)
         {
            for (float z = -0.0f; z <= 0.01f; z += 0.055f)
            {
               Color color = colors[count%colors.length];
//               Color color = Color.hsb(360.0 * random.nextDouble(), random.nextDouble(), random.nextDouble()); 
               Vector3D32 pointsOffset = new Vector3D32(x, y, 0 * RandomNumbers.nextFloat(random, -5.0f, 5.0f));
               meshBuilder.addCube(0.05f, pointsOffset, color);
               Box box = new Box(0.025f, 0.025f, 0.025f);
               box.setTranslateX(pointsOffset.getX());
               box.setTranslateY(pointsOffset.getY());
               box.setTranslateZ(0.05f + pointsOffset.getZ());
               box.setMaterial(new PhongMaterial(color));
               boxes.add(box);
               count++;
            }
         }
      }

      System.out.println("Number of boxes: " + count);
      return boxes;
   }

   private Color randomColor()
   {
      return Color.hsb(360.0 * random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
