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
import us.ihmc.commons.InterpolationTools;
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
   private final Point3D debugPosition = new Point3D();
   private final AtomicReference<PlanarRegionsList> planarRegions = new AtomicReference<>();

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

         if (ONLY_VISUALIZE_TOP)
         {
            meshBuilder.addPolygon(new RigidBodyTransform(new Quaternion(), new Point3D(x, y, height)), heightMapCell, computeColorFromHeight(height));
         }
         else
         {
            meshBuilder.addBox(gridResolutionXY,
                               gridResolutionXY,
                               renderedHeight,
                               new Point3D(x, y, heightMapMessage.getEstimatedGroundHeight() + 0.5 * renderedHeight),
                               computeColorFromHeight(height));
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

   public static Color computeColorFromHeight(double height)
   {
      // Using interpolation between key color points
      double r = 0, g = 0, b = 0;
      double redR = 1.0, redG = 0.0, redB = 0.0;
      double magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
      double orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
      double yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
      double blueR = 0.0, blueG = 0.0, blueB = 1.0;
      double greenR = 0.0, greenG = 1.0, greenB = 0.0;
      double gradientSize = 0.2;
      double gradientLength = 1.0;
      double alpha = height % gradientLength;
      if (alpha < 0)
         alpha = 1 + alpha;
      while (alpha > 5 * gradientSize)
         alpha -= 5 * gradientSize;

      if (alpha <= gradientSize * 1)
      {
         r = InterpolationTools.linearInterpolate(magentaR, blueR, (alpha) / gradientSize);
         g = InterpolationTools.linearInterpolate(magentaG, blueG, (alpha) / gradientSize);
         b = InterpolationTools.linearInterpolate(magentaB, blueB, (alpha) / gradientSize);
      }
      else if (alpha <= gradientSize * 2)
      {
         r = InterpolationTools.linearInterpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
         g = InterpolationTools.linearInterpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
         b = InterpolationTools.linearInterpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
      }
      else if (alpha <= gradientSize * 3)
      {
         r = InterpolationTools.linearInterpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
         g = InterpolationTools.linearInterpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
         b = InterpolationTools.linearInterpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
      }
      else if (alpha <= gradientSize * 4)
      {
         r = InterpolationTools.linearInterpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
         g = InterpolationTools.linearInterpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
         b = InterpolationTools.linearInterpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
      }
      else if (alpha <= gradientSize * 5)
      {
         r = InterpolationTools.linearInterpolate(orangeR, redR, (alpha - gradientSize * 4) / gradientSize);
         g = InterpolationTools.linearInterpolate(orangeG, redG, (alpha - gradientSize * 4) / gradientSize);
         b = InterpolationTools.linearInterpolate(orangeB, redB, (alpha - gradientSize * 4) / gradientSize);
      }
      else
      {
         throw new RuntimeException("no valid color");
      }

      if (r == 0.0 && g == 0.0 && b == 0.0)
         throw new RuntimeException("Shouldn't return black.)");
      return new Color((float) r, (float) g, (float) b, 1.0f);
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
