package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;

import java.util.function.BiConsumer;

public abstract class RDXProjectionShape
{
   protected final ImBoolean renderIfNoTexture = new ImBoolean(true);
   protected final ImBoolean hidden = new ImBoolean(false);
   protected ModelInstance modelInstance;
   protected Model model;
   protected Texture latestTexture;
   private Mesh mesh;

   public void setTextureCoordinateCalculator(BiConsumer<Point3DReadOnly[], TexCoord2f[]> textureCoordinateCalculator)
   {
   }

   public BiConsumer<Point3DReadOnly[], TexCoord2f[]> getTextureCoordinateCalculator()
   {
      return null;
   }

   public final void updateMeshLazy()
   {
      updateMeshLazy(null);
   }

   public abstract void updateMeshLazy(RigidBodyTransformReadOnly pose);

   public abstract void renderImGuiWidgets();

   public void updateTexture(Texture texture, float opacity)
   {
      if (this.latestTexture != null)
         this.latestTexture.dispose();
      this.latestTexture = texture;
      Material material = model.nodes.get(0).parts.get(0).material;
      material.set(new BlendingAttribute(opacity));
      if (texture != null)
         material.set(TextureAttribute.createDiffuse(texture));
      modelInstance = new ModelInstance(model);
   }

   protected void updateModelFromMeshDataHolder(MeshDataHolder meshDataHolder)
   {
      mesh = RDXMeshDataInterpreter.interpretMeshData(meshDataHolder);

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();
      if (latestTexture != null)
         material.set(TextureAttribute.createDiffuse(latestTexture));
      modelBuilder.part(meshPart, material);

      if (model != null)
         model.dispose();

      model = modelBuilder.end();

      modelInstance = new ModelInstance(model);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      boolean skipRenderables = false;
      if (!renderIfNoTexture.get() && latestTexture == null)
         skipRenderables = true;
      if (hidden.get())
         skipRenderables = true;

      if (modelInstance != null && !skipRenderables)
         modelInstance.getRenderables(renderables, pool);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
