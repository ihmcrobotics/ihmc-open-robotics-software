package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

public class RDXEdgeDefinedPolygonGraphic implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private ModelInstance modelInstance;
   private Model lastModel;
   private volatile Runnable buildMeshAndCreateModelInstance = null;
   private final Material material = new Material();
   private final ArrayList<RDXSplineBody> edgeLines = new ArrayList<>();
   private final Color edgeColor;
   private final float opacity;

   public RDXEdgeDefinedPolygonGraphic(Color color, Color edgeColor, float opacity)
   {
      this.opacity = opacity;
      this.edgeColor = edgeColor;
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(new com.badlogic.gdx.graphics.Color(color.r, color.g, color.b, 1.0f)));
      material.set(new BlendingAttribute(true, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA, opacity));
   }

   public void update()
   {
      if (buildMeshAndCreateModelInstance != null)
      {
         buildMeshAndCreateModelInstance.run();
         buildMeshAndCreateModelInstance = null;
      }
   }

   public void generateMeshAsync(Point3D[][] edges)
   {
      executorService.clearQueueAndExecute(() -> generateMesh(edges));
   }

   public synchronized void generateMesh(Point3D[][] edges)
   {
      int numSplines = edges.length;
      int numPointsPerSpline = edges[0].length;

      buildMeshAndCreateModelInstance = () ->
      {
         if (lastModel != null)
         {
            lastModel.dispose();
         }

         // generate the mesh
         ModelBuilder modelBuilder = new ModelBuilder();
         modelBuilder.begin();
         MeshPartBuilder meshBuilder = modelBuilder.part("mesh", GL20.GL_TRIANGLES, VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal, material);
         // create rectangle meshes that connect adjacent splines
         for (int i = 0; i < numSplines; i++)
         {
            // create edge lines to help with visualization of the complex resulting polygon mesh
            edgeLines.add(new RDXSplineBody(0.01f, opacity));
            edgeLines.get(i).setColor(edgeColor);

            // wrap around the index of the next spline to handle cases where adjacent splines do not connect at their endpoints
            int j = (i + 1) % numSplines;
            for (int k = 0; k < numPointsPerSpline - 1; k++)
            {
               //generate edge lines
               edgeLines.get(i).generateMeshes(edges[i][k], edges[i][k + 1]);

               // get the four points defining a rectangle
               Point3D p1 = edges[i][k];
               Point3D p2 = edges[i][k + 1];
               Point3D p3 = edges[j][k + 1];
               Point3D p4 = edges[j][k];

               // calculate the normal of the rectangle
               double nx = (p2.getY() - p1.getY()) * (p4.getZ() - p1.getZ()) - (p2.getZ() - p1.getZ()) * (p4.getY() - p1.getY());
               double ny = (p2.getZ() - p1.getZ()) * (p4.getX() - p1.getX()) - (p2.getX() - p1.getX()) * (p4.getZ() - p1.getZ());
               double nz = (p2.getX() - p1.getX()) * (p4.getY() - p1.getY()) - (p2.getY() - p1.getY()) * (p4.getX() - p1.getX());
               double length = Math.sqrt(nx * nx + ny * ny + nz * nz);
               nx /= length;
               ny /= length;
               nz /= length;

               // add the vertices and indices of the rectangle to the mesh
               meshBuilder.rect((float) p1.getX(),
                                (float) p1.getY(),
                                (float) p1.getZ(),
                                (float) p2.getX(),
                                (float) p2.getY(),
                                (float) p2.getZ(),
                                (float) p3.getX(),
                                (float) p3.getY(),
                                (float) p3.getZ(),
                                (float) p4.getX(),
                                (float) p4.getY(),
                                (float) p4.getZ(),
                                (float) nx,
                                (float) ny,
                                (float) nz);

               // add a second rectangle with the vertices in reverse order to create a face facing the opposite direction
               // this is to avoid having the mesh being visible only from one side
               meshBuilder.rect((float) p4.getX(),
                                (float) p4.getY(),
                                (float) p4.getZ(),
                                (float) p3.getX(),
                                (float) p3.getY(),
                                (float) p3.getZ(),
                                (float) p2.getX(),
                                (float) p2.getY(),
                                (float) p2.getZ(),
                                (float) p1.getX(),
                                (float) p1.getY(),
                                (float) p1.getZ(),
                                (float) -nx,
                                (float) -ny,
                                (float) -nz);
            }
         }

         // create the model instance
         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel);
      };
   }

   public void dispose()
   {
      executorService.destroy();
   }

   public void clear()
   {
      if (lastModel != null)
      {
         lastModel.dispose();
         lastModel = null;
      }
      modelInstance = null;
      for (RDXSplineBody edgeLine : edgeLines)
         edgeLine.clear();
      edgeLines.clear();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
      if (!edgeLines.isEmpty())
      {
         for (RDXSplineBody edgeLine : edgeLines)
            edgeLine.getRenderables(renderables, pool);
      }
   }
}