package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

public class RDXLineGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();

   private volatile Runnable buildMeshAndCreateModelInstance = null;

   private ModelInstance modelInstance;
   private Model lastModel;

   private Color color;
   private float lineWidth = 0.1f;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public RDXLineGraphic(float lineWidth, Color color)
   {
      this.color = color;
      this.lineWidth = lineWidth;
   }

   public void clear()
   {
      generateMeshes(new ArrayList<>(), 1);
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshesAsync(ArrayList<Point3D> points, int skip)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(points, skip));
   }

   /**
    * Generates a mesh for the given points. The points are sequentially pairwise connected as 0-1, 2-3, 4-5, etc. This is useful for visualizing
    * feature correspondences. For example, matching planar region pairs from two different sets of planar regions.
    * @param points The points to connect pairwise.
    */
   public synchronized void generateMeshForMatchLines(ArrayList<Point3DReadOnly> points)
   {
      // if we're passing in null, make an empty list
      if (points == null)
         points = new ArrayList<>();

      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
      for (int i = 0; i < points.size(); i+= 2)
      {
         meshBuilder.addLine(points.get(i), points.get(i+1), lineWidth, color);
      }
      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(new com.badlogic.gdx.graphics.Color(0.7f, 0.7f, 0.7f, 1.0f)));
         modelBuilder.part(meshPart, material);

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel);
      };
   }

   public synchronized void generateMeshes(ArrayList<Point3D> points, int skip)
   {
      // if we're passing in null, make an empty list
      if (points == null)
         points = new ArrayList<>();

      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
      for (int i = skip; i < points.size(); i+= skip)
      {
         meshBuilder.addLine(points.get(i - skip), points.get(i), lineWidth, color);
      }
      buildMeshAndCreateModelInstance = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(new com.badlogic.gdx.graphics.Color(0.7f, 0.7f, 0.7f, 1.0f)));
         modelBuilder.part(meshPart, material);

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel);
      };
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
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
