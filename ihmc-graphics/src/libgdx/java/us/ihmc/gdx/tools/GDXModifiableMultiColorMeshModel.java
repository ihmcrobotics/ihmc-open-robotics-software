package us.ihmc.gdx.tools;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import org.lwjgl.opengl.GL41;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;

import java.util.function.Consumer;

public class GDXModifiableMultiColorMeshModel
{
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
   private Model model;
   private final Texture paletteTexture = GDXMultiColorMeshBuilder.loadPaletteTexture();
   private final Material material = new Material();
   private final TextureAttribute diffuseTexture = TextureAttribute.createDiffuse(paletteTexture);
   private final ColorAttribute diffuseColor = ColorAttribute.createDiffuse(Color.WHITE);

   public GDXModifiableMultiColorMeshModel()
   {
      material.set(diffuseTexture);
      material.set(diffuseColor);
   }

   public void begin()
   {
      meshBuilder.clear();
   }

   /**
    * Use this or access the mesh builder via the getter.
    */
   public void build(Consumer<GDXMultiColorMeshBuilder> buildModel)
   {
      buildModel.accept(meshBuilder);
   }

   public void end()
   {
      Mesh mesh = meshBuilder.generateMesh(); // TODO: Do this without allocating

      if (model == null)
      {
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         modelBuilder.begin();
         modelBuilder.part(meshPart, material);
         model = modelBuilder.end();
      }
      else
      {
         model.nodes.get(0).parts.get(0).meshPart.mesh = mesh;
         model.nodes.get(0).parts.get(0).meshPart.size = mesh.getNumIndices();
      }
   }

   public GDXMultiColorMeshBuilder getMeshBuilder()
   {
      return meshBuilder;
   }

   public Model getModel()
   {
      return model;
   }
}
