package us.ihmc.footstepPlanning.ui.viewers;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.HeightMapMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotics.heightMap.HeightMapTools;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class HeightMapVisualizer extends AnimationTimer
{
   private final Color heightMapColor;
   private final Color groundPlaneColor;
   private final Group rootNode = new Group();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final AtomicReference<Pair<Mesh, Material>> heightMapToRender = new AtomicReference<>();
   private final MeshView heightMapMeshView = new MeshView();

   private final ExecutorService meshComputation = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean processing = new AtomicBoolean(false);
   private final AtomicDouble maxHeightToVisualize = new AtomicDouble(Double.NaN);

   public HeightMapVisualizer()
   {
      rootNode.getChildren().add(heightMapMeshView);

      Color olive = Color.OLIVE;
      Color blue = Color.BLUE;
      heightMapColor = Color.color(olive.getRed(), olive.getGreen(), olive.getBlue(), 0.95f);
      groundPlaneColor = Color.color(blue.getRed(), blue.getGreen(), blue.getBlue(), 0.8).brighter();
   }

   public void setMaxHeight(double maxHeight)
   {
      this.maxHeightToVisualize.set(maxHeight);
   }

   public void update(HeightMapMessage data)
   {
      if (!processing.getAndSet(true))
      {
         meshComputation.execute(() -> computeMesh(data));
      }
   }

   @Override
   public void handle(long now)
   {
      Pair<Mesh, Material> heightMapMesh = heightMapToRender.getAndSet(null);
      if (heightMapMesh != null)
      {
         heightMapMeshView.setMesh(heightMapMesh.getKey());
         heightMapMeshView.setMaterial(heightMapMesh.getValue());
      }
   }

   private void computeMesh(HeightMapMessage heightMapMessage)
   {
      /* Compute mesh */
      meshBuilder.clear();
      IDLSequence.Float heights = heightMapMessage.getHeights();
      double gridResolutionXY = heightMapMessage.getXyResolution();
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getGridSizeXy(), gridResolutionXY);

      for (int i = 0; i < heights.size(); i++)
      {
         int xIndex = HeightMapTools.keyToXIndex(heightMapMessage.getKeys().get(i), centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(heightMapMessage.getKeys().get(i), centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, heightMapMessage.getGridCenterX(), gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, heightMapMessage.getGridCenterY(), gridResolutionXY, centerIndex);
         double height = heights.get(i);
         double renderedHeight = height - heightMapMessage.getEstimatedGroundHeight();

         meshBuilder.addBox(gridResolutionXY, gridResolutionXY, renderedHeight, new Point3D(x, y, heightMapMessage.getEstimatedGroundHeight() + 0.5 * renderedHeight), heightMapColor);
      }

      double renderedGroundPlaneHeight = 0.005;
      meshBuilder.addBox(heightMapMessage.getGridSizeXy(),
                         heightMapMessage.getGridSizeXy(),
                         renderedGroundPlaneHeight,
                         new Point3D(heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY(), heightMapMessage.getEstimatedGroundHeight()),
                         groundPlaneColor);

      double maxHeight = maxHeightToVisualize.get();
      if (!Double.isNaN(maxHeight))
      {
         meshBuilder.addCube(0.05, 0.0, 0.0, maxHeight, Color.BLACK);
      }

      heightMapToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      processing.set(false);
   }

   public void clear()
   {
      meshBuilder.clear();
      heightMapToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   @Override
   public void stop()
   {
      super.stop();
      meshComputation.shutdownNow();
   }

   public Group getRoot()
   {
      return rootNode;
   }
}
