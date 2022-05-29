package us.ihmc.gdx.visualizers;

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
import controller_msgs.msg.dds.HeightMapMessage;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;
import java.util.function.Supplier;

public class GDXHeightMapGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private volatile Runnable buildMeshAndCreateModelInstance = null;
   private ModelInstance modelInstance;
   private Model lastModel;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private boolean renderGroundPlane = true;

   public void setRenderGroundPlane(boolean renderGroundPlane)
   {
      this.renderGroundPlane = renderGroundPlane;
   }

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
      LogTools.info("Receiving height map with " + heightMapMessage.getKeys().size() + " cells, ground plane at " + heightMapMessage.getEstimatedGroundHeight());

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

      generateMeshes(heightProvider,
                     keyProvider,
                     heightMapMessage.getHeights().size(),
                     heightMapMessage.getXyResolution(),
                     heightMapMessage.getGridSizeXy(),
                     heightMapMessage.getGridCenterX(),
                     heightMapMessage.getGridCenterY(),
                     heightMapMessage.getEstimatedGroundHeight());
   }


   public synchronized void generateMeshes(IntToDoubleFunction heightsProvider,
                                           IntFunction<Integer> keysProvider,
                                           int numberOfOccupiedCells,
                                           double gridResolutionXY,
                                           double gridSizeXy,
                                           double gridCenterX,
                                           double gridCenterY,
                                           double groundHeight)
   {
      List<GDXMultiColorMeshBuilder> meshBuilders = new ArrayList<>();
      Color olive = Color.OLIVE;
      Color blue = Color.BLUE;

      if (Double.isNaN(groundHeight))
         groundHeight = 0.0;

      int centerIndex = HeightMapTools.computeCenterIndex(gridSizeXy, gridResolutionXY);

      for (int i = 0; i < numberOfOccupiedCells; i++)
      {
         GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
         int key = keysProvider.apply(i);
         int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
         int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);
         double x = HeightMapTools.indexToCoordinate(xIndex, gridCenterX, gridResolutionXY, centerIndex);
         double y = HeightMapTools.indexToCoordinate(yIndex, gridCenterY, gridResolutionXY, centerIndex);
         double height = heightsProvider.applyAsDouble(i);
         double renderedHeight = height - groundHeight + 0.02;

         meshBuilder.addBox(gridResolutionXY, gridResolutionXY, renderedHeight, new Point3D(x, y, groundHeight + 0.5 * renderedHeight), olive);
         meshBuilders.add(meshBuilder);
      }

      if (renderGroundPlane)
      {
         GDXMultiColorMeshBuilder groundMeshBuilder = new GDXMultiColorMeshBuilder();
         double renderedGroundPlaneHeight = 0.005;
         groundMeshBuilder.addBox(gridSizeXy,
                                  gridSizeXy,
                                  renderedGroundPlaneHeight,
                                  new Point3D(gridCenterX, gridCenterY, groundHeight),
                                  blue);
         meshBuilders.add(groundMeshBuilder);
      }

      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Material material = new Material();
         Texture paletteTexture = GDXMultiColorMeshBuilder.loadPaletteTexture();
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
}
