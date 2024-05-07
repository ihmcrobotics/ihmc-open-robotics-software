package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.MeshDataGeneratorMissing;

import java.util.function.BiConsumer;

public class RDXProjectionHemisphere extends RDXProjectionShape
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble xRadius = new ImDouble(1);
   private final ImDouble yRadius = new ImDouble(1);
   private final ImDouble zRadius = new ImDouble(1);
   private final ImInt latitudeResolution = new ImInt(128);
   private final ImInt longitudeResolution = new ImInt(128);
   private final ImDouble fieldOfView = new ImDouble(200.0);
   private boolean rebuildMesh = true;

   private BiConsumer<Point3DReadOnly[], TexCoord2f[]> textureCoordinateCalculator = null;

   @Override
   public void renderImGuiWidgets()
   {
      rebuildMesh |= ImGui.inputDouble(labels.get("X Radius"), xRadius);
      rebuildMesh |= ImGui.inputDouble(labels.get("Y Radius"), yRadius);
      rebuildMesh |= ImGui.inputDouble(labels.get("Z Radius"), zRadius);
      rebuildMesh |= ImGui.inputInt(labels.get("Latitude Resolution"), latitudeResolution);
      rebuildMesh |= ImGui.inputInt(labels.get("Longitude Resolution"), longitudeResolution);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection field of View"), fieldOfView, 10.0, 200.0);
   }

   private final RigidBodyTransform pose = new RigidBodyTransform();
   private final RigidBodyTransform lastPose = new RigidBodyTransform();

   /**
    * Update the mesh of the shape. This method is called lazily, meaning that the mesh is only updated if the pose has changed.
    *
    * @param newPose the pose of the camera with respect to the mid-point of the 2 cameras.
    */
   @Override
   public void updateMeshLazy(RigidBodyTransformReadOnly newPose)
   {
      if (newPose != null && !newPose.epsilonEquals(lastPose, 1.0e-7))
      {
         rebuildMesh = true;
      }

      if (!rebuildMesh)
         return;

      rebuildMesh = false;

      MeshDataHolder meshDataHolder = MeshDataGeneratorMissing.InvertedHemiEllipsoidNoBottom((float) xRadius.get(),
                                                                                             (float) yRadius.get(),
                                                                                             (float) zRadius.get(),
                                                                                             latitudeResolution.get(),
                                                                                             longitudeResolution.get(),
                                                                                             (float) Math.toRadians(fieldOfView.get()));
      { // Rotate the hemisphere to have the camera pointing forward assuming the depth axis is positive x.
         Quaternion rotation = new Quaternion();
         rotation.appendPitchRotation(0.5 * Math.PI);
         meshDataHolder = MeshDataHolder.rotate(meshDataHolder, rotation);
      }

      if (newPose != null)
      {
         pose.set(newPose);
         lastPose.set(newPose);
         meshDataHolder.applyTransform(pose);
      }

      if (textureCoordinateCalculator != null)
      {
         textureCoordinateCalculator.accept(meshDataHolder.getVertices(), meshDataHolder.getTexturePoints());
      }

      if (newPose != null)
      {
         meshDataHolder.applyInverseTransform(pose);
      }

      updateModelFromMeshDataHolder(meshDataHolder);
   }

   public void setXRadius(double xRadius)
   {
      if (xRadius != this.xRadius.get())
      {
         this.xRadius.set(xRadius);
         rebuildMesh = true;
      }
   }

   public void setYRadius(double yRadius)
   {
      if (yRadius != this.yRadius.get())
      {
         this.yRadius.set(yRadius);
         rebuildMesh = true;
      }
   }

   public void setZRadius(double zRadius)
   {
      if (zRadius != this.zRadius.get())
      {
         this.zRadius.set(zRadius);
         rebuildMesh = true;
      }
   }

   public void setLatitudeResolution(int latitudeResolution)
   {
      if (latitudeResolution != this.latitudeResolution.get())
      {
         this.latitudeResolution.set(latitudeResolution);
         rebuildMesh = true;
      }
   }

   public void setLongitudeResolution(int longitudeResolution)
   {
      if (longitudeResolution != this.longitudeResolution.get())
      {
         this.longitudeResolution.set(longitudeResolution);
         rebuildMesh = true;
      }
   }

   public void setFieldOfView(double fieldOfView)
   {
      if (fieldOfView != this.fieldOfView.get())
      {
         this.fieldOfView.set(fieldOfView);
         rebuildMesh = true;
      }
   }

   public double getXRadius()
   {
      return xRadius.get();
   }

   public double getYRadius()
   {
      return yRadius.get();
   }

   public double getZRadius()
   {
      return zRadius.get();
   }

   public int getLatitudeResolution()
   {
      return latitudeResolution.get();
   }

   public int getLongitudeResolution()
   {
      return longitudeResolution.get();
   }

   public double getFieldOfView()
   {
      return fieldOfView.get();
   }

   @Override
   public void setTextureCoordinateCalculator(BiConsumer<Point3DReadOnly[], TexCoord2f[]> textureCoordinateCalculator)
   {
      this.textureCoordinateCalculator = textureCoordinateCalculator;
      rebuildMesh = true;
   }

   @Override
   public BiConsumer<Point3DReadOnly[], TexCoord2f[]> getTextureCoordinateCalculator()
   {
      return textureCoordinateCalculator;
   }

   public static class FisheyeTextureCalculator implements BiConsumer<Point3DReadOnly[], TexCoord2f[]>
   {
      private final CameraOrientation cameraOrientation;
      private final double fx;
      private final double fy;
      private final double cx;
      private final double cy;
      private final double k1;
      private final double k2;
      private final double k3;
      private final double k4;
      private final double fov;

      private final double tx_min, tx_max, ty_min, ty_max;

      private boolean mirrorX = false;
      private boolean mirrorY = false;

      public enum CameraOrientation
      {
         /**
          * Assumes z-forward, y-down, x-right
          */
         Z_DEPTH_POSITIVE,
         /**
          * Assumes x-forward, y-left, z-up
          */
         X_DEPTH_POSITIVE
      }

      public FisheyeTextureCalculator(CameraOrientation cameraOrientation,
                                      double fx,
                                      double fy,
                                      double cx,
                                      double cy,
                                      double k1,
                                      double k2,
                                      double k3,
                                      double k4,
                                      double fov)
      {
         this.cameraOrientation = cameraOrientation;
         this.fx = fx;
         this.fy = fy;
         this.cx = cx;
         this.cy = cy;
         this.k1 = k1;
         this.k2 = k2;
         this.k3 = k3;
         this.k4 = k4;
         this.fov = fov;

         double theta_max = 0.5 * fov;
         double theta_d_max = thetaDistortion(theta_max);
         tx_max = theta_d_max * fx + cx;
         tx_min = -theta_d_max * fx + cx;
         ty_max = theta_d_max * fy + cy;
         ty_min = -theta_d_max * fy + cy;
      }

      @Override
      public void accept(Point3DReadOnly[] vertices, TexCoord2f[] texturePoints)
      {
         for (int i = 0; i < vertices.length; i++)
         {
            Point3DReadOnly point3DReadOnly = vertices[i];

            if (point3DReadOnly.distanceFromOriginSquared() < 1.0e-7)
            {
               texturePoints[i].set(0.5f, 0.5f);
               continue;
            }

            double w, h, d;
            if (cameraOrientation == CameraOrientation.Z_DEPTH_POSITIVE)
            {
               w = point3DReadOnly.getX();
               h = point3DReadOnly.getY();
               d = point3DReadOnly.getZ();
            }
            else if (cameraOrientation == CameraOrientation.X_DEPTH_POSITIVE)
            {
               w = -point3DReadOnly.getY();
               h = -point3DReadOnly.getZ();
               d = point3DReadOnly.getX();
            }
            else
            {
               throw new RuntimeException("Unexpected camera orientation: " + cameraOrientation);
            }

            TexCoord2f texturePoint = texturePoints[i];

            double r = EuclidCoreTools.norm(w, h);
            double a = w / r;
            double b = h / r;
            double theta = Math.atan2(r, d);

            if (theta > 0.5 * fov)
            {
               texturePoint.set(0.0f, 0.0f);
               continue;
            }

            double theta_d = thetaDistortion(theta);
            double tx = theta_d * a * fx + cx;
            double ty = theta_d * b * fy + cy;

            tx = (tx - tx_min) / (tx_max - tx_min);
            ty = (ty - ty_min) / (ty_max - ty_min);

            if (mirrorX)
               tx = 1.0 - tx;
            if (mirrorY)
               ty = 1.0 - ty;

            texturePoint.set((float) tx, (float) ty);
         }
      }

      private double thetaDistortion(double theta)
      {
         double theta2 = theta * theta;
         double theta4 = theta2 * theta2;
         double theta6 = theta4 * theta2;
         double theta8 = theta6 * theta2;
         return theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
      }

      public void setMirrorX(boolean mirrorX)
      {
         this.mirrorX = mirrorX;
      }

      public void setMirrorY(boolean mirrorY)
      {
         this.mirrorY = mirrorY;
      }

      public double getFocalLengthX()
      {
         return fx;
      }

      public double getFocalLengthY()
      {
         return fy;
      }

      public double getPrinciplePointX()
      {
         return cx;
      }

      public double getPrinciplePointY()
      {
         return cy;
      }

      public double getK1()
      {
         return k1;
      }

      public double getK2()
      {
         return k2;
      }

      public double getK3()
      {
         return k3;
      }

      public double getK4()
      {
         return k4;
      }

      public double getFieldOfView()
      {
         return fov;
      }

      public boolean isMirrorX()
      {
         return mirrorX;
      }

      public boolean isMirrorY()
      {
         return mirrorY;
      }
   }
}
