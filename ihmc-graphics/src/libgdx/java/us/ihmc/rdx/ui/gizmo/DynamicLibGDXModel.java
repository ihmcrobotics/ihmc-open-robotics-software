package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * TODO: This class needs to probably be several implementations of some interfaces.
 * It's a mess right now.
 */
public class DynamicLibGDXModel
{
   private Model model;
   private final Node node;
   private MeshPart meshPart;
   private ModelInstance modelInstance;
   private Material material;
   private Supplier<Mesh> buildMesh;
   private Mesh mesh;
   private boolean needsRebuild = true;
   private boolean customModel = false;

   private final RigidBodyTransform localTransform = new RigidBodyTransform();

   public DynamicLibGDXModel()
   {
      model = new Model();
      node = new Node();
      model.nodes.add(node);
      node.id = "node" + model.nodes.size;
   }

   public void setModel(Model model)
   {
      this.model = model;
      customModel = true;
   }

   public void setMesh(Consumer<RDXMultiColorMeshBuilder> buildMesh)
   {
      this.buildMesh = () ->
      {
         RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
         buildMesh.accept(meshBuilder);
         return meshBuilder.generateMesh();
      };
      needsRebuild = true;
   }

   public void setMesh(Supplier<Mesh> buildMesh)
   {
      this.buildMesh = buildMesh;
      needsRebuild = true;
   }

   public void setMesh(Mesh mesh)
   {
      this.mesh = mesh;
      this.buildMesh = null;
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

   public void buildIfNeeded()
   {
      if (needsRebuild && !customModel)
      {
         needsRebuild = false;
         if (mesh != null && buildMesh != null)
         {
            node.parts.removeIndex(0);
            dispose();
         }

         if (buildMesh != null)
            mesh = buildMesh.get();
         if (mesh != null)
         {
            meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            node.parts.add(new NodePart(meshPart, material));
         }
      }
   }

   public void dispose()
   {
      if (mesh != null)
         mesh.dispose();
   }

   public ModelInstance newModelInstance()
   {
      buildIfNeeded();
      return new ModelInstance(model);
   }

   public Model getModel()
   {
      return model;
   }

   public ModelInstance getOrCreateModelInstance()
   {
      if (needsRebuild || modelInstance == null)
      {
         modelInstance = newModelInstance();
      }
      return modelInstance;
   }

   public RigidBodyTransform getLocalTransform()
   {
      return localTransform;
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
