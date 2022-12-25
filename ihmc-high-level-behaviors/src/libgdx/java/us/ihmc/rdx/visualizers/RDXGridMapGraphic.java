package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import org.ejml.data.DMatrixRMaj;
import org.lwjgl.opengl.GL41;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;

public class RDXGridMapGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private volatile Runnable buildMeshAndCreateModelInstance = null;
   private ModelInstance modelInstance;
   private Model lastModel;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public void clear()
   {
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshesAsync(HeightMapMessage heightMapMessage)
   {
      LogTools.debug("Receiving height map with {} cells, ground plane at {}", heightMapMessage.getKeys().size(), heightMapMessage.getEstimatedGroundHeight());

      if (!isGeneratingMeshes.getAndSet(true))
      {
         executorService.clearQueueAndExecute(() -> generateMeshes(heightMapMessage));
      }
   }

   private final AtomicBoolean isGeneratingMeshes = new AtomicBoolean();

   public synchronized void generateMeshes(HeightMapMessage heightMapMessage)
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
                     heightMapMessage.getEstimatedGroundHeight());
   }

   RotationMatrix rotationMatrix = new RotationMatrix();

   public synchronized void generateMeshes(IntToDoubleFunction heightsProvider,
                                           IntToDoubleFunction variancesProvider,
                                           IntFunction<Integer> keysProvider,
                                           IntFunction<Vector3DReadOnly> normalsProvider,
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

      for (int i = 0; i < numberOfOccupiedCells; i++)
      {
         RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
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

         // FIXME scale the color based on some metric
         meshBuilder.addPolygon(Arrays.asList(topLeft, topRight, bottomRight, bottomLeft), Color.OLIVE);
         meshBuilders.add(meshBuilder);
      }

      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Material material = new Material();
         Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
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
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      }

      ;

      isGeneratingMeshes.set(false);
   }

   public Color computeColor(Color noVariance, Color highVariance, double alpha)
   {
      float r = (float) InterpolationTools.linearInterpolate(noVariance.r, highVariance.r, alpha);
      float g = (float) InterpolationTools.linearInterpolate(noVariance.g, highVariance.g, alpha);
      float b = (float) InterpolationTools.linearInterpolate(noVariance.b, highVariance.b, alpha);
      float a = (float) InterpolationTools.linearInterpolate(noVariance.a, highVariance.a, alpha);

      return new Color(r, g, b, a);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
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
