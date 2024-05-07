package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import imgui.ImGui;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXProjectionRectangle extends RDXProjectionShape
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble aspectRatio = new ImDouble(16.0 / 9.0);
   private final ImDouble shapeWidth = new ImDouble(1);
   private final ImInt textureWidthResolution = new ImInt(0);
   private boolean rebuildMesh = false;

   @Override
   public void renderImGuiWidgets()
   {
      rebuildMesh |= ImGui.inputInt(labels.get("Texture: Width resolution"), textureWidthResolution);
      rebuildMesh |= ImGuiTools.sliderDouble(labels.get("AspectRatio"), aspectRatio, 0.1, 2.0);
      rebuildMesh |= ImGui.checkbox(labels.get("Render if no texture"), renderIfNoTexture);
      rebuildMesh |= ImGui.checkbox(labels.get("Hidden"), hidden);
   }

   @Override
   public void updateMeshLazy(RigidBodyTransformReadOnly pose)
   {
      if (!rebuildMesh)
         return;

      rebuildMesh = false;

      double height = shapeWidth.get() / aspectRatio.get();
      MeshDataHolder meshDataHolder;
      double xMin = -0.5 * shapeWidth.get();
      double yMin = -0.5 * height;
      double xMax = 0.5 * shapeWidth.get();
      double yMax = 0.5 * height;

      int xVertices = getTextureWidthResolution();
      int yVertices = getTextureHeightResolution();
      int numberOfVertices = xVertices * yVertices;

      if (xVertices > 2 && yVertices > 2 && numberOfVertices <= MeshBuilder.MAX_VERTICES)
      {
         Point3D32[] points = new Point3D32[numberOfVertices];
         Vector3D32[] normals = new Vector3D32[numberOfVertices];
         TexCoord2f[] textPoints = new TexCoord2f[numberOfVertices];

         for (int x = 0; x < xVertices; x++)
         {
            for (int y = 0; y < yVertices; y++)
            {
               double xCoordinate = xMin + (xMax - xMin) * x / (xVertices - 1);
               double yCoordinate = yMin + (yMax - yMin) * y / (yVertices - 1);
               points[x * yVertices + y] = new Point3D32((float) xCoordinate, (float) yCoordinate, 0.0f);
               normals[x * yVertices + y] = new Vector3D32(0.0f, 0.0f, 1.0f);
            }
         }

         for (int x = 0; x < xVertices; x++)
         {
            for (int y = 0; y < yVertices; y++)
            {
               textPoints[x * yVertices + y] = new TexCoord2f((float) x / (xVertices - 1), (float) y / (yVertices - 1));
            }
         }

         int[] triangleIndices = new int[6 * (xVertices - 1) * (yVertices - 1)];
         int index = 0;

         for (int x = 0; x < xVertices - 1; x++)
         {
            for (int y = 0; y < yVertices - 1; y++)
            {
               triangleIndices[index++] = x * yVertices + y;
               triangleIndices[index++] = (x + 1) * yVertices + y;
               triangleIndices[index++] = x * yVertices + y + 1;

               triangleIndices[index++] = x * yVertices + y + 1;
               triangleIndices[index++] = (x + 1) * yVertices + y;
               triangleIndices[index++] = (x + 1) * yVertices + y + 1;
            }
         }

         meshDataHolder = new MeshDataHolder(points, textPoints, triangleIndices, normals);
      }
      else
      {
         meshDataHolder = MeshDataGenerator.FlatRectangle(xMin, yMin, xMax, yMax, 0.0);
      }

      Quaternion rotation = new Quaternion();
      rotation.appendPitchRotation(Math.PI / 2.0);
      rotation.appendYawRotation(-Math.PI / 2.0);
      meshDataHolder = MeshDataHolder.rotate(meshDataHolder, rotation);

      updateModelFromMeshDataHolder(meshDataHolder);
   }

   public void setShapeWidth(double widthInMeters)
   {
      if (this.shapeWidth.get() != widthInMeters && widthInMeters > 0.0)
      {
         this.shapeWidth.set(widthInMeters);
         rebuildMesh = true;
      }
   }

   public void setTextureWidthResolution(int nPixelsWidth)
   {
      if (this.textureWidthResolution.get() != nPixelsWidth && nPixelsWidth > 1)
      {
         this.textureWidthResolution.set(nPixelsWidth);
         rebuildMesh = true;
      }
   }

   public int getNumberOfPixelsWidth()
   {
      return textureWidthResolution.get();
   }

   public double getShapeWidth()
   {
      return shapeWidth.get();
   }

   public void setAspectRatio(double aspectRatio)
   {
      if (this.aspectRatio.get() != aspectRatio && aspectRatio > 0.0)
      {
         this.aspectRatio.set(aspectRatio);
         rebuildMesh = true;
      }
   }

   public double getAspectRatio()
   {
      return aspectRatio.get();
   }

   public int getTextureWidthResolution()
   {
      return textureWidthResolution.get();
   }

   public int getTextureHeightResolution()
   {
      return (int) Math.round(textureWidthResolution.get() / aspectRatio.get());
   }
}
