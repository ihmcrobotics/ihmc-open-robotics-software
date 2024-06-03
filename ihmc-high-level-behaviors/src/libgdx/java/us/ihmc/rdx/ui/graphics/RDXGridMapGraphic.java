package us.ihmc.rdx.ui.graphics;

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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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

/**
 * This class has been replaced with a newer implementation called {@link RDXHeightMapGraphicNew}, please use that going forward and use this one only for
 * reference
 */
@Deprecated
public class RDXGridMapGraphic implements RenderableProvider
{
   private static final int MAX_INDEX_MESH_BUILDER = MeshBuilder.MAX_INDEX / 4; // To prevent over flowing, stay far away from MAX_INDEX

   private final ModelBuilder modelBuilder = new ModelBuilder();

   private Model lastModel;
   private Texture paletteTexture = null;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private static boolean colorFromHeight = true;
   private static boolean inPaintHeight = true;
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

   public void setColorFromHeight(boolean colorFromHeight)
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

      generateMeshes(heightProvider,
                     keyProvider,
                     heightMapMessage.getHeights().size(),
                     heightMapMessage.getXyResolution(),
                     heightMapMessage.getGridSizeXy(),
                     heightMapMessage.getGridCenterX(),
                     heightMapMessage.getGridCenterY(),
                     heightMapMessage.getEstimatedGroundHeight(),
                     heightMapMessage.getSequenceId());
   }

   private void generateMeshes(IntToDoubleFunction heightsProvider,
                               IntFunction<Integer> keysProvider,
                               int numberOfOccupiedCells,
                               double gridResolutionXY,
                               double gridSizeXy,
                               double gridCenterX,
                               double gridCenterY,
                               double groundHeight,
                               long id)
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
                                                                        renderGroundCells);

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
                                                                     boolean renderGroundCells)
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

      // Fill in all the vertices for the mesh texture
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

      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
      int currentIndexForMeshBuilder = 0; // This is used to prevent us from over filling a MeshBuilder

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
               if (colorFromHeight)
                  color = HeightMapTools.computeGDXColorFromHeight(0.5 * (maxHeight + minHeight));
               else
                  color = idColorFunction.getColor((int) id);

               meshBuilder.addPolygon(Arrays.asList(topLeft, topRight, bottomRight, bottomLeft), color);
               currentIndexForMeshBuilder++;

               // Create a new MeshBuilder if this one gets filled up
               if (currentIndexForMeshBuilder >= MAX_INDEX_MESH_BUILDER)
               {
                  meshBuilders.add(meshBuilder);
                  meshBuilder = new RDXMultiColorMeshBuilder();
                  currentIndexForMeshBuilder = 0;
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
            if (colorFromHeight)
               color = HeightMapTools.computeGDXColorFromHeight(0.5 * (maxHeight + minHeight));
            else
               color = idColorFunction.getColor((int) id);

            meshBuilder.addPolygon(Arrays.asList(topLeft, topRight, bottomRight, bottomLeft), color);
            currentIndexForMeshBuilder++;

            // Create a new MeshBuilder if this one gets filled up
            if (currentIndexForMeshBuilder >= MAX_INDEX_MESH_BUILDER)
            {
               meshBuilders.add(meshBuilder);
               meshBuilder = new RDXMultiColorMeshBuilder();
               currentIndexForMeshBuilder = 0;
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
      groundMeshBuilder.addBox(gridSizeXy,
                               gridSizeXy,
                               renderedGroundPlaneHeight,
                               new Point3D(gridCenterX, gridCenterY, groundHeight),
                               HeightMapTools.computeGDXColorFromHeight(groundHeight));

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

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      ModelInstance modelInstance = latestModel.get();
      // Sync over current and add
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
