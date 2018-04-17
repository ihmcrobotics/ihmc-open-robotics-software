package us.ihmc.octoMap;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;

import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.stage.Stage;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class SearchNodeNeighborsVisualizer extends Application
{
   private static final Color NEIGHBOR_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.0);
   private final List<OcTreeKey> neighbors;
   private final int depth = 13;
   private final double resolution = 0.025;
   private final int treeDepth = 16;
   private final double searchRadius = 0.50;

   public SearchNodeNeighborsVisualizer()
   {
      double nodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
      Point3D nodeCenter = new Point3D(nodeSize, nodeSize, nodeSize);
      nodeCenter.scale(0.5);

      OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(nodeCenter, depth, resolution, treeDepth);
      neighbors = OcTreeKeyTools.computeNeighborKeys(key, depth, resolution, treeDepth, searchRadius);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      View3DFactory view3dFactory = new View3DFactory(800, 600);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      for (int i = 0; i < neighbors.size(); i++)
      {
         OcTreeKey ocTreeKey = neighbors.get(i);
         double boxSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
         Point3D boxCenter = OcTreeKeyConversionTools.keyToCoordinate(ocTreeKey, depth, resolution, treeDepth);
         addFreeBox(boxSize, boxCenter, view3dFactory.getRoot());
      }

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public void addFreeBox(double size, Point3D center, Group root)
   {
      addBox(size, center, root, NEIGHBOR_COLOR);
   }

   private void addBox(double size, Point3D center, Group root, Color color)
   {
      Box box = new Box(size, size, size);
      box.setTranslateX(center.getX());
      box.setTranslateY(center.getY());
      box.setTranslateZ(center.getZ());
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(color);
      box.setMaterial(material);
      root.getChildren().add(box);
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
