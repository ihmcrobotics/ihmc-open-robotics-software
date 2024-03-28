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
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RDXProjectionSphere
{
   private ModelInstance modelInstance;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble sphereRadius = new ImDouble(1);
   private final ImInt sphereLatitudeVertices = new ImInt(100);
   private final ImInt sphereLongitudeVertices = new ImInt(100);
   private final ImDouble focalLengthX = new ImDouble();
   private final ImDouble focalLengthY = new ImDouble();
   private final ImDouble principlePointX = new ImDouble(0.0);
   private final ImDouble principlePointY = new ImDouble(0.5);
   private final ImBoolean renderSphereIfNoTexture = new ImBoolean(true);
   private final ImBoolean hidden = new ImBoolean(false);
   private Model model;
   private final Vector3D vertexRay = new Vector3D();
   private Mesh mesh;
   private Texture latestTexture;
   private boolean rebuildMesh = false;

   public void create()
   {
      rebuildUVSphereMesh();
   }

   public void renderImGuiWidgets()
   {
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Sphere radius"), sphereRadius, 0.2, 5);
      rebuildMesh |= ImGuiTools.volatileInputInt(labels.get("Sphere latitude vertices"), sphereLatitudeVertices);
      rebuildMesh |= ImGuiTools.volatileInputInt(labels.get("Sphere longitude vertices"), sphereLongitudeVertices);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale X"), focalLengthX, 0.01, 2.0);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale Y"), focalLengthY, 0.01, 2.0);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Principle point X (Cx)"), principlePointX, -0.5, 0.5);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Principle point Y (Cy)"), principlePointY, -0.5, 0.5);
      rebuildMesh |= ImGui.checkbox(labels.get("Render sphere if no texture"), renderSphereIfNoTexture);
      rebuildMesh |= ImGui.checkbox(labels.get("Hidden"), hidden);

      if (rebuildMesh)
         rebuildUVSphereMesh();

      rebuildMesh = false;
   }

   /** Only needs to be done when parameters change, not the texture. */
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

   public void updateTexture(Texture texture, float opacity)
   {
      if (this.latestTexture != null)
         this.latestTexture.dispose();
      this.latestTexture = texture;
      Material material = model.nodes.get(0).parts.get(0).material;
      material.set(new BlendingAttribute(opacity));
      material.set(TextureAttribute.createDiffuse(texture));
      modelInstance = new ModelInstance(model);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      boolean skipRenderables = false;
      if (!renderSphereIfNoTexture.get() && latestTexture == null)
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

   public double getRadius()
   {
      return this.sphereRadius.get();
   }

   public void setRadius(double radius)
   {

      if (this.sphereRadius.get() != radius)
      {
         this.sphereRadius.set(radius);
         rebuildMesh = true;
      }
   }

   public double getPrinciplePointX()
   {
      return principlePointX.get();
   }

   public void setPrinciplePointX(double principlePointX)
   {
      if (this.principlePointX.get() != principlePointX)
      {
         this.principlePointX.set(principlePointX);
         rebuildMesh = true;
      }
   }

   public double getPrinciplePointY()
   {
      return principlePointY.get();
   }

   public void setPrinciplePointY(double principlePointY)
   {
      if (this.principlePointY.get() != principlePointY)
      {
         this.principlePointY.set(principlePointY);
         rebuildMesh = true;
      }
   }

   public double getFocalLengthX()
   {
      return focalLengthX.get();
   }

   public void setFocalLengthX(double focalLengthX)
   {
      if (this.focalLengthX.get() != focalLengthX)
      {
         this.focalLengthX.set(focalLengthX);
         rebuildMesh = true;
      }
   }

   public double getFocalLengthY()
   {
      return focalLengthY.get();
   }

   public void setFocalLengthY(double focalLengthY)
   {
      if (this.focalLengthY.get() != focalLengthY)
      {
         this.focalLengthY.set(focalLengthY);
         rebuildMesh = true;
      }
   }
}
