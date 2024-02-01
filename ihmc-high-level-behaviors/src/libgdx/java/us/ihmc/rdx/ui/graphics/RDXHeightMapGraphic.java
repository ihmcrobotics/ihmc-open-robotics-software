package us.ihmc.rdx.ui.graphics;

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
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;

public class RDXHeightMapGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private volatile Runnable buildMeshAndCreateModelInstance = null;
   private ModelInstance modelInstance;
   private Model lastModel;
   private Texture paletteTexture = null;
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ImBoolean renderGroundPlane = new ImBoolean(true);
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
      Color perfectColor = Color.TEAL;
      Color terribleColor = Color.RED;
      Color blue = Color.BLUE;

      if (Double.isNaN(groundHeight))
         groundHeight = 0.0;

      int centerIndex = HeightMapTools.computeCenterIndex(gridSizeXy, gridResolutionXY);

      ConvexPolygon2DReadOnly localPoints = generatePointsForHeightPatch(gridResolutionXY);

      for (int i = 0; i < numberOfOccupiedCells; i++)
      {
         RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
         int key = keysProvider.apply(i);
         int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, gridCenterX, gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, gridCenterY, gridResolutionXY, centerIndex);
         double height = heightsProvider.applyAsDouble(i);
//         double renderedHeight = height - groundHeight + 0.02;
         double renderedHeight = height + 0.02;

         Vector3DReadOnly normal = null;
         if (normalsProvider != null)
            normal = normalsProvider.apply(i);
         Color color = Color.OLIVE;
         if (variancesProvider != null)
            color = computeColor(perfectColor, terribleColor, MathTools.clamp(Math.abs(variancesProvider.applyAsDouble(i) / 0.05), 0.0, 1.0));
         RigidBodyTransform transform = generateTransformForHeightPatch(x, y, renderedHeight, normal);
         if (!transform.containsNaN())
         {
            transformToWorld.transform(transform);

            if (!transform.getRotation().geometricallyEquals(rotationMatrix, Math.PI / 2.0))
            {
               transform.getRotation().appendPitchRotation(Math.PI);
            }

            meshBuilder.addPolygon(transform, localPoints, color);
            meshBuilders.add(meshBuilder);
         }
      }

      if (renderGroundPlane.get())
      {
         RDXMultiColorMeshBuilder groundMeshBuilder = new RDXMultiColorMeshBuilder();
         double renderedGroundPlaneHeight = 0.005;
         groundMeshBuilder.addBox(gridSizeXy, gridSizeXy, renderedGroundPlaneHeight, new Point3D(gridCenterX, gridCenterY, groundHeight), blue);
         meshBuilders.add(groundMeshBuilder);
      }

      buildMeshAndCreateModelInstance = () ->
      {
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
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };

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

   public ConvexPolygon2DBasics generatePointsForHeightPatch(double resolution)
   {
      ConvexPolygon2DBasics polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.5 * resolution, 0.5 * resolution));
      polygon.addVertex(new Point2D(0.5 * resolution, -0.5 * resolution));
      polygon.addVertex(new Point2D(-0.5 * resolution, -0.5 * resolution));
      polygon.addVertex(new Point2D(-0.5 * resolution, 0.5 * resolution));
      polygon.update();

      return polygon;
   }

   public RigidBodyTransform generateTransformForHeightPatch(double x, double y, double z, Vector3DReadOnly normal)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(x, y, z);
      if (normal != null)
         transform.getRotation().set(EuclidGeometryTools.axisAngleFromZUpToVector3D(normal));

      return transform;
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

   public void setRenderGroundPlane(boolean renderGroundPlane)
   {
      this.renderGroundPlane.set(renderGroundPlane);
   }

   public ImBoolean getRenderGroundPlane()
   {
      return renderGroundPlane;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }
}
