package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.MeshDataGeneratorMissing;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXProjectionSphere extends RDXProjectionShape
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble sphereRadius = new ImDouble(1);
   private final ImInt sphereLatitudeVertices = new ImInt(100);
   private final ImInt sphereLongitudeVertices = new ImInt(100);
   private final ImDouble focalLengthX = new ImDouble();
   private final ImDouble focalLengthY = new ImDouble();
   private final ImDouble principlePointX = new ImDouble(0.0);
   private final ImDouble principlePointY = new ImDouble(0.5);
   private final Vector3D vertexRay = new Vector3D();
   private boolean rebuildMesh = false;

   @Override
   public void renderImGuiWidgets()
   {
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Sphere radius"), sphereRadius, 0.2, 5);
      rebuildMesh |= ImGuiTools.volatileInputInt(labels.get("Sphere latitude vertices"), sphereLatitudeVertices);
      rebuildMesh |= ImGuiTools.volatileInputInt(labels.get("Sphere longitude vertices"), sphereLongitudeVertices);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale X"), focalLengthX, 0.01, 2.0);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale Y"), focalLengthY, 0.01, 2.0);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Principle point X (Cx)"), principlePointX, -0.5, 0.5);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Principle point Y (Cy)"), principlePointY, -0.5, 0.5);
      rebuildMesh |= ImGui.checkbox(labels.get("Render sphere if no texture"), renderIfNoTexture);
      rebuildMesh |= ImGui.checkbox(labels.get("Hidden"), hidden);
   }

   @Override
   public void updateMeshLazy(RigidBodyTransformReadOnly pose)
   {
      if (!rebuildMesh)
         return;

      rebuildMesh = false;

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

      updateModelFromMeshDataHolder(sphereMeshDataHolder);
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
