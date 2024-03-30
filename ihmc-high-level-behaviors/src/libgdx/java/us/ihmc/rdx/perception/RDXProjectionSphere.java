package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.MeshDataGeneratorMissing;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXProjectionSphere
{
   private ModelInstance modelInstance;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble sphereRadius = new ImDouble(20.0);
   private final ImInt sphereLatitudeVertices = new ImInt(100);
   private final ImInt sphereLongitudeVertices = new ImInt(100);
   private final ImBoolean syncProjectionScales = new ImBoolean(false);
   private final ImDouble focalLengthX = new ImDouble(0.560838);
   private final ImDouble focalLengthY = new ImDouble(1.0);
   private final ImDouble principlePointX = new ImDouble(0.0);
   private final ImDouble principlePointY = new ImDouble(0.0);
   private final ImBoolean renderSphereIfNoTexture = new ImBoolean(true);
   private Model model;
   private final Vector3D vertexRay = new Vector3D();
   private Mesh mesh;
   private Texture latestTexture;

   public void create()
   {
      rebuildUVSphereMesh();
   }

   public void renderImGuiWidgets()
   {
      boolean rebuildMesh = false;
      rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Sphere radius"), sphereRadius);
      rebuildMesh |= ImGuiTools.volatileInputInt(labels.get("Sphere latitude vertices"), sphereLatitudeVertices);
      rebuildMesh |= ImGuiTools.volatileInputInt(labels.get("Sphere longitude vertices"), sphereLongitudeVertices);
      rebuildMesh |= ImGui.checkbox(labels.get("Sync projection scales"), syncProjectionScales);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale X"), focalLengthX, 0.01, 2.0);
      if (syncProjectionScales.get())
      {
         ImGui.beginDisabled();
         focalLengthY.set(focalLengthX.get());
      }
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale Y"), focalLengthY, 0.01, 2.0);
      if (syncProjectionScales.get())
      {
         ImGui.endDisabled();
      }
      rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Principle point X (Cx)"), principlePointX);
      rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Principle point Y (Cy)"), principlePointY);
      rebuildMesh |= ImGui.checkbox(labels.get("Render sphere if no texture"), renderSphereIfNoTexture);

      if (rebuildMesh)
         rebuildUVSphereMesh();
   }

   /** Only needs to be done when parameters change, not the texture.*/
   public void rebuildUVSphereMesh()
   {
      MeshDataHolder sphereMeshDataHolder = MeshDataGeneratorMissing.InvertedSphere(sphereRadius.get(),
                                                                                    sphereLatitudeVertices.get(),
                                                                                    sphereLongitudeVertices.get());

      for (int i = 0; i < sphereMeshDataHolder.getVertices().length; i++)
      {
         Point3D32 vertex = sphereMeshDataHolder.getVertices()[i];
         TexCoord2f texturePoint = sphereMeshDataHolder.getTexturePoints()[i];

         vertexRay.set(vertex);

         double angleOfIncidence = EuclidCoreMissingTools.angleFromFirstToSecondVector3D(Axis3D.X, vertexRay);
         double azimuthalAngle = Math.atan2(-vertex.getZ(), -vertex.getY());


         double imageX = principlePointX.get() + focalLengthX.get() * angleOfIncidence * Math.cos(azimuthalAngle);
         double imageY = principlePointY.get() + focalLengthY.get() * angleOfIncidence * Math.sin(azimuthalAngle);

         texturePoint.setX(imageX + 0.5);
         texturePoint.setY(imageY + 0.0);
      }

      mesh = RDXMeshDataInterpreter.interpretMeshData(sphereMeshDataHolder);

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

   public void updateTexture(Texture texture)
   {
      if (this.latestTexture != null)
         this.latestTexture.dispose();
      this.latestTexture = texture;
      Material material = model.nodes.get(0).parts.get(0).material;
      material.set(TextureAttribute.createDiffuse(texture));
      modelInstance = new ModelInstance(model);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      boolean skipRenderables = false;
      if (!renderSphereIfNoTexture.get() && latestTexture == null)
         skipRenderables = true;

      if (modelInstance != null && !skipRenderables)
         modelInstance.getRenderables(renderables, pool);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public ImDouble getPrinciplePointX()
   {
      return principlePointX;
   }

   public ImDouble getPrinciplePointY()
   {
      return principlePointY;
   }
}
