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
   private final Group rootNode = new Group();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final AtomicReference<Pair<Mesh, Material>> heightMapToRender = new AtomicReference<>();
   private final MeshView heightMapMeshView = new MeshView();

   private static final double renderedZeroHeight = -1.5;

   private final double gridResolutionXY;
   private final int minMaxIndexXY;

   private final ExecutorService meshComputation = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean processing = new AtomicBoolean(false);

   public HeightMapVisualizer()
   {
      rootNode.getChildren().add(heightMapMeshView);

      Color color = Color.OLIVE;
      heightMapColor = Color.color(color.getRed(), color.getGreen(), color.getBlue(), 0.7);

      HeightMapParameters parameters = new HeightMapParameters();
      gridResolutionXY = parameters.getGridResolutionXY();
      minMaxIndexXY = HeightMapTools.toIndex(parameters.getGridSizeXY(), gridResolutionXY, 0);
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

   private void computeMesh(HeightMapMessage heightMapData)
   {
      LogTools.info("Computing mesh for " + heightMapData.getXCells().size() + " cells");

      /* Compute mesh */
      meshBuilder.clear();
      TIntArrayList xCells = heightMapData.getXCells();
      TIntArrayList yCells = heightMapData.getYCells();
      IDLSequence.Float heights = heightMapData.getHeights();

      for (int i = 0; i < xCells.size(); i++)
      {
         double x = (xCells.get(i) - minMaxIndexXY) * gridResolutionXY;
         double y = (yCells.get(i) - minMaxIndexXY) * gridResolutionXY;
         double height = heights.get(i);

         double renderedHeight = Math.max(0.0, height - renderedZeroHeight);

         meshBuilder.addBox(gridResolutionXY, gridResolutionXY, renderedHeight, new Point3D(x, y, renderedZeroHeight + 0.5 * renderedHeight), heightMapColor);
      }

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
