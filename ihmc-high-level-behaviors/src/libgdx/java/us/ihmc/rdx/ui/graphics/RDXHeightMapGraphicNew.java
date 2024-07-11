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
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalMapTile;
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
 * Creates a graphic for the GPU Height Map to be visualized in the RDX UI. Each height value from the height map is turned into a 2cm polygon that is then
 * visualized on the UI. The height value location will be in the center of the 2cm polygon that is visualized.
 */
public class RDXHeightMapGraphicNew implements RenderableProvider
{
   private static final int MAX_INDEX_MESH_BUILDER = MeshBuilder.MAX_INDEX / 4; // To prevent over flowing, stay far away from MAX_INDEX

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private final AtomicReference<List<RDXMultiColorMeshBuilder>> latestMeshBuilder = new AtomicReference<>(null);
   private final AtomicReference<ModelInstance> latestModel = new AtomicReference<>(null);

   private Model lastModel;
   private Texture paletteTexture = null;

   public void update()
   {
      createModelFromMeshBuilders();
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
                     heightMapMessage.getEstimatedGroundHeight());
   }

   public void generateMeshesGlobal(GlobalMapTile globalMapTile)
   {
      IntToDoubleFunction heightsProvider = globalMapTile::getHeight;
      IntFunction<Integer> keysProvider = globalMapTile::getKey;


      List<RDXMultiColorMeshBuilder> meshBuilders = generateHeightCells(heightsProvider,
                                                                        keysProvider,
                                                                        1,
                                                                        2,
                                                                        globalMapTile.getGridSizeXY(),
                                                                        globalMapTile.getCenterX(),
                                                                        globalMapTile.getCenterY(),
                                                                        globalMapTile.getHeight(0));
      latestMeshBuilder.set(meshBuilders);
   }

   private void generateMeshes(IntToDoubleFunction heightsProvider,
                               IntFunction<Integer> keysProvider,
                               int numberOfOccupiedCells,
                               double gridResolutionXY,
                               double gridSizeXy,
                               double gridCenterX,
                               double gridCenterY,
                               double groundHeight)
   {
      List<RDXMultiColorMeshBuilder> meshBuilders = generateHeightCells(heightsProvider,
                                                                        keysProvider,
                                                                        numberOfOccupiedCells,
                                                                        gridResolutionXY,
                                                                        gridSizeXy,
                                                                        gridCenterX,
                                                                        gridCenterY,
                                                                        groundHeight);
      latestMeshBuilder.set(meshBuilders);
   }

   private static List<RDXMultiColorMeshBuilder> generateHeightCells(IntToDoubleFunction heightsProvider,
                                                                     IntFunction<Integer> keysProvider,
                                                                     int numberOfOccupiedCells,
                                                                     double gridResolutionXY,
                                                                     double gridSizeXy,
                                                                     double gridCenterX,
                                                                     double gridCenterY,
                                                                     double groundHeight)
   {
      List<RDXMultiColorMeshBuilder> meshBuilders = new ArrayList<>();

      int centerIndex = HeightMapTools.computeCenterIndex(gridSizeXy, gridResolutionXY);
      int cellsPerAxis = 2 * centerIndex + 1;

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

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            Point3D topLeft = new Point3D();
            Point3D topRight = new Point3D();
            Point3D bottomLeft = new Point3D();
            Point3D bottomRight = new Point3D();

            // Take each vertex and create a 2cm polygon where the vertex is in the center
            double halfWidth = gridResolutionXY / 2.0;
            Point3DReadOnly vertexCenter = vertices[xIndex][yIndex];
            topLeft.set(vertexCenter.getX() + halfWidth, vertexCenter.getY() + halfWidth, vertexCenter.getZ());
            topRight.set(vertexCenter.getX() - halfWidth, vertexCenter.getY() + halfWidth, vertexCenter.getZ());
            bottomLeft.set(vertexCenter.getX() - halfWidth, vertexCenter.getY() - halfWidth, vertexCenter.getZ());
            bottomRight.set(vertexCenter.getX() + halfWidth, vertexCenter.getY() - halfWidth, vertexCenter.getZ());

            double[] redGreenBlue = HeightMapTools.getRedGreenBlue(vertexCenter.getZ());
            Color color = new Color((float) redGreenBlue[0], (float) redGreenBlue[1], (float) redGreenBlue[2], 1.0f);

            meshBuilder.addPolygon(Arrays.asList(topLeft, topRight, bottomLeft, bottomRight), color);
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

      return meshBuilders;
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