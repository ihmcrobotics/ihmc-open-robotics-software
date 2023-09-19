package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.mesh.RDXIDMappedColorFunction;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;

public class RDXGridMapGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private Model lastModel;
   private Texture paletteTexture = null;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private boolean colorFromHeight = true;
   private boolean inPaintHeight = true;
   private boolean renderGroundCells = false;
   private boolean renderGroundPlane = false;

   private static final RDXIDMappedColorFunction idColorFunction = new RDXIDMappedColorFunction();

   private final AtomicReference<List<RDXMultiColorMeshBuilder>> latestMeshBuilder = new AtomicReference<>(null);
   private final AtomicReference<ModelInstance> latestModel = new AtomicReference<>(null);

   private static final double zForInPainting = 0.95;

   public void clear()
   {
   }

   public void update()
   {
      createModelFromMeshBuilders();
   }

   public void colorFromHeight(boolean colorFromHeight)
   {
      this.colorFromHeight = colorFromHeight;
   }

   public void setInPaintHeight(boolean inPaintHeight)
   {
      this.inPaintHeight = inPaintHeight;
   }

   public void setRenderGroundPlane(boolean renderGroundPlane)
   {
      this.renderGroundPlane = renderGroundPlane;
   }

   public void setRenderGroundCells(boolean renderGroundCells)
   {
      this.renderGroundCells = renderGroundCells;
   }

   public void generateMeshesAsync(HeightMapMessage heightMapMessage)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(heightMapMessage));
   }

   public void generateMeshes(HeightMapMessage heightMapMessage)
   {
      IntToDoubleFunction heightProvider = (d) -> (double) heightMapMessage.getHeights().get(d);
      IntFunction<Integer> keyProvider = (d) -> heightMapMessage.getKeys().get(d);
      IntFunction<Vector3DReadOnly> normalsProvider;
      IntToDoubleFunction variancesProvider;
      if (heightMapMessage.getNormals().size() != heightMapMessage.getHeights().size())
         normalsProvider = null;
      else
         normalsProvider = (d) -> heightMapMessage.getNormals().get(d);
      if (heightMapMessage.getVariances().size() != heightMapMessage.getHeights().size())
         variancesProvider = null;
      else
         variancesProvider = (d) -> heightMapMessage.getVariances().get(d);

      generateMeshes(heightProvider,
                     variancesProvider,
                     keyProvider,
                     normalsProvider,
                     heightMapMessage.getHeights().size(),
                     heightMapMessage.getXyResolution(),
                     heightMapMessage.getGridSizeXy(),
                     heightMapMessage.getGridCenterX(),
                     heightMapMessage.getGridCenterY(),
                     heightMapMessage.getEstimatedGroundHeight(),
                     heightMapMessage.getSequenceId(),
                     inPaintHeight);
   }

   private void generateMeshes(IntToDoubleFunction heightsProvider,
                               IntToDoubleFunction variancesProvider,
                               IntFunction<Integer> keysProvider,
                               IntFunction<Vector3DReadOnly> normalsProvider,
                               int numberOfOccupiedCells,
                               double gridResolutionXY,
                               double gridSizeXy,
                               double gridCenterX,
                               double gridCenterY,
                               double groundHeight,
                               long id,
                               boolean inPaintHeight)
   {
      List<RDXMultiColorMeshBuilder> meshBuilders = generateHeightCells(heightsProvider,
                                                                        keysProvider,
                                                                        numberOfOccupiedCells,
                                                                        gridResolutionXY,
                                                                        gridSizeXy,
                                                                        gridCenterX,
                                                                        gridCenterY,
                                                                        groundHeight,
                                                                        id,
                                                                        inPaintHeight,
                                                                        renderGroundCells,
                                                                        colorFromHeight);

      if (renderGroundPlane)
         meshBuilders.add(generateGroundPlaneMesh(gridSizeXy, gridCenterX, gridCenterY, groundHeight));

      latestMeshBuilder.set(meshBuilders);
   }

   private static List<RDXMultiColorMeshBuilder> generateHeightCells(IntToDoubleFunction heightsProvider,
                                                                     IntFunction<Integer> keysProvider,
                                                                     int numberOfOccupiedCells,
                                                                     double gridResolutionXY,
                                                                     double gridSizeXy,
                                                                     double gridCenterX,
                                                                     double gridCenterY,
                                                                     double groundHeight,
                                                                     long id,
                                                                     boolean inPaintHeight,
                                                                     boolean renderGroundCells,
                                                                     boolean computeColorFromHeight)
   {
      List<RDXMultiColorMeshBuilder> meshBuilders = new ArrayList<>();

      int centerIndex = HeightMapTools.computeCenterIndex(gridSizeXy, gridResolutionXY);
      int cellsPerAxis = 2 * centerIndex + 1;

      // fixme how many triangles do I have?
      Point3D[][] vertices = new Point3D[cellsPerAxis][];
      for (int i = 0; i < cellsPerAxis; i++)
      {
         vertices[i] = new Point3D[cellsPerAxis];
      }

      for (int i = 0; i < numberOfOccupiedCells; i++)
      {
         int key = keysProvider.apply(i);
         int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, gridCenterX, gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, gridCenterY, gridResolutionXY, centerIndex);
         double height = heightsProvider.applyAsDouble(i);

         vertices[xIndex][yIndex] = new Point3D(x, y, height);
      }

      // fill in all the vertices for the mesh texture
      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            if (vertices[xIndex][yIndex] == null)
            {
               double x = HeightMapTools.indexToCoordinate(xIndex, gridCenterX, gridResolutionXY, centerIndex);
               double y = HeightMapTools.indexToCoordinate(yIndex, gridCenterY, gridResolutionXY, centerIndex);
               vertices[xIndex][yIndex] = new Point3D(x, y, groundHeight);
            }
         }
      }

      int maxCellsPerBuilder = MeshBuilder.MAX_INDEX / 6;
      int cellsPerBuilder = 0;
      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();

      if (renderGroundCells)
      {
         for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
         {
            for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
            {
               if (xIndex > cellsPerAxis - 2)
                  continue;
               if (yIndex > cellsPerAxis - 2)
                  continue;

               Point3DReadOnly topLeft = vertices[xIndex][yIndex];
               Point3DReadOnly topRight = vertices[xIndex + 1][yIndex];
               Point3DReadOnly bottomRight = vertices[xIndex + 1][yIndex + 1];
               Point3DReadOnly bottomLeft = vertices[xIndex][yIndex + 1];

               double maxHeight = max(topLeft.getZ(), topRight.getZ(), bottomLeft.getZ(), bottomRight.getZ());
               double minHeight = min(topLeft.getZ(), topRight.getZ(), bottomLeft.getZ(), bottomRight.getZ());

               if (!inPaintHeight)
               {
                  double distanceZ = maxHeight - minHeight;
                  double distanceZ2 = distanceZ * distanceZ;
                  double distanceX = Math.abs(topLeft.getX() - topRight.getX());
                  double normalizedZSquared = distanceZ2 / (distanceZ2 + distanceX * distanceX);
                  if (zForInPainting * zForInPainting < normalizedZSquared)
                     continue;
               }

               Color color;
               if (computeColorFromHeight)
                  color = computeColorFromHeight(0.5 * (maxHeight + minHeight));
               else
                  color = idColorFunction.getColor((int) id);

               meshBuilder.addPolygon(Arrays.asList(topLeft, topRight, bottomRight, bottomLeft), color);
               cellsPerBuilder++;

               if (cellsPerBuilder >= maxCellsPerBuilder)
               {
                  meshBuilders.add(meshBuilder);
                  meshBuilder = new RDXMultiColorMeshBuilder();
                  cellsPerBuilder = 0;
               }
            }
         }
         meshBuilders.add(meshBuilder);
      }
      else
      {
         for (int i = 0; i < numberOfOccupiedCells; i++)
         {
            int key = keysProvider.apply(i);
            int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
            int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);

            if (xIndex > cellsPerAxis - 2)
               continue;
            if (yIndex > cellsPerAxis - 2)
               continue;

            Point3DReadOnly topLeft = vertices[xIndex][yIndex];
            Point3DReadOnly topRight = vertices[xIndex + 1][yIndex];
            Point3DReadOnly bottomRight = vertices[xIndex + 1][yIndex + 1];
            Point3DReadOnly bottomLeft = vertices[xIndex][yIndex + 1];

            double maxHeight = max(topLeft.getZ(), topRight.getZ(), bottomLeft.getZ(), bottomRight.getZ());
            double minHeight = min(topLeft.getZ(), topRight.getZ(), bottomLeft.getZ(), bottomRight.getZ());

            if (!inPaintHeight)
            {
               double distanceZ = maxHeight - minHeight;
               double distanceZ2 = distanceZ * distanceZ;
               double distanceX = Math.abs(topLeft.getX() - topRight.getX());
               double normalizedZSquared = distanceZ2 / (distanceZ2 + distanceX * distanceX);
               if (zForInPainting * zForInPainting < normalizedZSquared)
                  continue;
            }

            Color color;
            if (computeColorFromHeight)
               color = computeColorFromHeight(0.5 * (maxHeight + minHeight));
            else
               color = idColorFunction.getColor((int) id);

            meshBuilder.addPolygon(Arrays.asList(topLeft, topRight, bottomRight, bottomLeft), color);
            cellsPerBuilder++;

            if (cellsPerBuilder >= maxCellsPerBuilder)
            {
               meshBuilders.add(meshBuilder);
               meshBuilder = new RDXMultiColorMeshBuilder();
               cellsPerBuilder = 0;
            }
         }
         meshBuilders.add(meshBuilder);
      }

      return meshBuilders;
   }

   private static RDXMultiColorMeshBuilder generateGroundPlaneMesh(double gridSizeXy, double gridCenterX, double gridCenterY, double groundHeight)
   {
      RDXMultiColorMeshBuilder groundMeshBuilder = new RDXMultiColorMeshBuilder();
      double renderedGroundPlaneHeight = 0.005;
      groundMeshBuilder.addBox(gridSizeXy, gridSizeXy, renderedGroundPlaneHeight, new Point3D(gridCenterX, gridCenterY, groundHeight), computeColorFromHeight(groundHeight));

      return groundMeshBuilder;
   }

   private void createModelFromMeshBuilders()
   {
      List<RDXMultiColorMeshBuilder> meshBuilders = latestMeshBuilder.getAndSet(null);

      if (meshBuilders == null)
         return;

      modelBuilder.begin();
      Material material = new Material();
      if (paletteTexture == null)
         paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(new Color(0.7f, 0.7f, 0.7f, 1.0f)));

      for (int i = 0; i < meshBuilders.size(); i++)
      {
         Mesh mesh = meshBuilders.get(i).generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         modelBuilder.part(meshPart, material);
      }

      if (lastModel != null)
         lastModel.dispose();

      lastModel = modelBuilder.end();
      ModelInstance modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model

      latestModel.set(modelInstance);
   }

   private static double max(double... values)
   {
      double height = Double.MIN_VALUE;
      for (double val : values)
      {
         if (Double.isFinite(val))
            height = Math.max(height, val);
      }

      return height;
   }

   private static double min(double... values)
   {
      double height = Double.MAX_VALUE;
      for (double val : values)
      {
         if (Double.isFinite(val))
            height = Math.min(height, val);
      }

      return height;
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
         alpha -=  5 * gradientSize;

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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      ModelInstance modelInstance = latestModel.get();
      // sync over current and add
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }
}
