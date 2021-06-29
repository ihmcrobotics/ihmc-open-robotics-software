package us.ihmc.gdx.ui.gizmo;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import org.lwjgl.opengl.GL32;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;

import java.util.function.Consumer;

public class DynamicGDXModel
{
   private final Model model;
   private final Node node;
   private MeshPart meshPart;
   private ModelInstance modelInstance;
   private Material material;
   private Consumer<GDXMultiColorMeshBuilder> buildMesh;
   private Mesh mesh;
   private boolean needsRebuild = true;

   public DynamicGDXModel()
   {
      model = new Model();
      node = new Node();
      model.nodes.add(node);
      node.id = "node" + model.nodes.size;
   }

   public void setMesh(Consumer<GDXMultiColorMeshBuilder> buildMesh)
   {
      this.buildMesh = buildMesh;
      needsRebuild = true;
   }

   public void invalidateMesh()
   {
      needsRebuild = true;
   }

   public void setMaterial(Material material)
   {
      if (this.material == null || !this.material.equals(material))
      {
         this.material = material;
         needsRebuild = true;
      }
   }

   private void buildIfNeeded()
   {
      if (needsRebuild)
      {
         needsRebuild = false;
         if (mesh != null)
         {
            node.parts.removeIndex(0);
            dispose();
         }

         GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
         buildMesh.accept(meshBuilder);
         mesh = meshBuilder.generateMesh();
         meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
         node.parts.add(new NodePart(meshPart, material));
      }
   }

   public void dispose()
   {
      mesh.dispose();
   }

   public ModelInstance newModelInstance()
   {
      buildIfNeeded();
      return new ModelInstance(model);
   }

   public ModelInstance getOrCreateModelInstance()
   {
      if (needsRebuild || modelInstance == null)
      {
         modelInstance = newModelInstance();
      }
      return modelInstance;
   }
}
