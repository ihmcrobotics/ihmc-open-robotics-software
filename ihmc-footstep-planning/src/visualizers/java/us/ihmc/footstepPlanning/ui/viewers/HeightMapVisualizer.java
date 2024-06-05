package us.ihmc.footstepPlanning.ui.viewers;

import com.google.common.util.concurrent.AtomicDouble;
import perception_msgs.msg.dds.HeightMapMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class HeightMapVisualizer extends AnimationTimer
{
   private static final boolean RENDER_BORDERS = false;
   private static final boolean RENDER_MAX_HEIGHT = false;
   private static final boolean RENDER_DEBUG_POSITION = false;
   private static final boolean ONLY_VISUALIZE_TOP = true;
   private static final boolean SHOW_GROUND_PLANE = false;

   private final Color groundPlaneColor;
   private final Group rootNode = new Group();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final AtomicReference<Pair<Mesh, Material>> heightMapToRender = new AtomicReference<>();
   private final MeshView heightMapMeshView = new MeshView();

   private final ExecutorService meshComputation = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean processing = new AtomicBoolean(false);
   private final AtomicDouble maxHeightToVisualize = new AtomicDouble(Double.NaN);
   private final Point3D debugPosition = new Point3D();
   private final AtomicReference<PlanarRegionsList> planarRegions = new AtomicReference<>();

   public HeightMapVisualizer()
   {
      rootNode.getChildren().add(heightMapMeshView);

      Color olive = Color.OLIVE;
      Color blue = Color.BLUE;
      groundPlaneColor = Color.color(blue.getRed(), blue.getGreen(), blue.getBlue(), 0.8).brighter();
   }

   public void setMaxHeight(double maxHeight)
   {
      this.maxHeightToVisualize.set(maxHeight);
   }

   public void setDebugPosition(int i, double position)
   {
      this.debugPosition.setElement(i, position);
   }

   public void update(HeightMapMessage data)
   {
      if (!processing.getAndSet(true))
      {
         meshComputation.execute(() -> computeMesh(data));
      }
   }

   public void setRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegions.set(planarRegions);
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

      if (Double.isNaN(heightMapMessage.getEstimatedGroundHeight()))
      {
         heightMapMessage.setEstimatedGroundHeight(heights.min());
      }

      ConvexPolygon2D heightMapCell = new ConvexPolygon2D();
      heightMapCell.addVertex(-0.5 * gridResolutionXY, -0.5 * gridResolutionXY);
      heightMapCell.addVertex(-0.5 * gridResolutionXY, 0.5 * gridResolutionXY);
      heightMapCell.addVertex(0.5 * gridResolutionXY, -0.5 * gridResolutionXY);
      heightMapCell.addVertex(0.5 * gridResolutionXY, 0.5 * gridResolutionXY);
      heightMapCell.update();

      for (int i = 0; i < heights.size(); i++)
      {
         int xIndex = HeightMapTools.keyToXIndex(heightMapMessage.getKeys().get(i), centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(heightMapMessage.getKeys().get(i), centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, heightMapMessage.getGridCenterX(), gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, heightMapMessage.getGridCenterY(), gridResolutionXY, centerIndex);
         double height = heights.get(i);
         double renderedHeight = height - heightMapMessage.getEstimatedGroundHeight();
         double[] redGreenBlue = HeightMapTools.getRedGreenBlue(height);

         if (ONLY_VISUALIZE_TOP)
         {
            meshBuilder.addPolygon(new RigidBodyTransform(new Quaternion(), new Point3D(x, y, height)),
                                   heightMapCell,
                                   new Color((float) redGreenBlue[0], (float) redGreenBlue[1], (float) redGreenBlue[2], 1.0f));
         }
         else
         {
            meshBuilder.addBox(gridResolutionXY,
                               gridResolutionXY,
                               renderedHeight,
                               new Point3D(x, y, heightMapMessage.getEstimatedGroundHeight() + 0.5 * renderedHeight),
                               new Color((float) redGreenBlue[0], (float) redGreenBlue[1], (float) redGreenBlue[2], 1.0f));
         }

         if (RENDER_BORDERS)
         {
            for (double dx : new double[] {-1.0, 1.0})
            {
               for (double dy : new double[] {-1.0, 1.0})
               {
                  Point3D start = new Point3D(x, y, heightMapMessage.getEstimatedGroundHeight());
                  Point3D end = new Point3D(x, y, height);
                  start.add(dx * 0.5 * gridResolutionXY, dy * 0.5 * gridResolutionXY, 0.0);
                  end.add(dx * 0.5 * gridResolutionXY, dy * 0.5 * gridResolutionXY, 0.0);
                  meshBuilder.addLine(start, end, 2e-3, Color.BLACK);
               }
            }
         }
      }

      if (SHOW_GROUND_PLANE)
      {
         double renderedGroundPlaneHeight = 0.005;
         meshBuilder.addBox(heightMapMessage.getGridSizeXy(),
                            heightMapMessage.getGridSizeXy(),
                            renderedGroundPlaneHeight,
                            new Point3D(heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY(), heightMapMessage.getEstimatedGroundHeight()),
                            groundPlaneColor);
      }

      if (RENDER_MAX_HEIGHT)
      {
         double maxHeight = maxHeightToVisualize.get();
         if (!Double.isNaN(maxHeight))
         {
            meshBuilder.addCube(0.05, 0.0, 0.0, maxHeight, Color.BLACK);
         }
      }

      if (RENDER_DEBUG_POSITION)
      {
         meshBuilder.addCube(0.05, debugPosition, Color.ORANGE);
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
