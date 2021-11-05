package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.HeightMapMessage;
import gnu.trove.list.array.TIntArrayList;
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
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

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

   public HeightMapVisualizer()
   {
      rootNode.getChildren().add(heightMapMeshView);

      Color olive = Color.OLIVE;
      Color blue = Color.BLUE;
      heightMapColor = Color.color(olive.getRed(), olive.getGreen(), olive.getBlue(), 0.7);
      groundPlaneColor = Color.color(blue.getRed(), blue.getGreen(), blue.getBlue(), 0.8).brighter();
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
      int minMaxIndex = HeightMapTools.minMaxIndex(heightMapMessage.getGridSizeXy(), gridResolutionXY);

      for (int i = 0; i < heights.size(); i++)
      {
         int xIndex = HeightMapTools.xIndex(heightMapMessage.getCells().get(i), minMaxIndex);
         int yIndex = HeightMapTools.yIndex(heightMapMessage.getCells().get(i), minMaxIndex);
         double x = HeightMapTools.toCoordinate(xIndex, heightMapMessage.getGridCenterX(), gridResolutionXY, minMaxIndex);
         double y = HeightMapTools.toCoordinate(yIndex, heightMapMessage.getGridCenterY(), gridResolutionXY, minMaxIndex);
         double height = heights.get(i);

         double renderedHeight = Math.max(0.0, height - heightMapMessage.getEstimatedGroundHeight());

         meshBuilder.addBox(gridResolutionXY, gridResolutionXY, renderedHeight, new Point3D(x, y, heightMapMessage.getEstimatedGroundHeight() + 0.5 * renderedHeight), heightMapColor);
      }

      double renderedGroundPlaneHeight = 0.01;
      meshBuilder.addBox(heightMapMessage.getGridSizeXy(),
                         heightMapMessage.getGridSizeXy(),
                         renderedGroundPlaneHeight,
                         new Point3D(heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY(), heightMapMessage.getEstimatedGroundHeight()),
                         groundPlaneColor);

      heightMapToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      processing.set(false);
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
