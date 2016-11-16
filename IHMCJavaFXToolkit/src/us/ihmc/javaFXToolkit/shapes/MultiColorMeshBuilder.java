package us.ihmc.javaFXToolkit.shapes;

import static us.ihmc.javaFXToolkit.shapes.MeshBuilder.DEFAULT_RES;

import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import us.ihmc.graphics3DDescription.MeshDataGenerator;
import us.ihmc.graphics3DDescription.MeshDataHolder;

public class MultiColorMeshBuilder
{
   private final MeshBuilder meshBuilder = new MeshBuilder();
   private TextureColorPalette colorPalette;

   public MultiColorMeshBuilder()
   {
      colorPalette = new TextureColorPalette2D();
   }

   public MultiColorMeshBuilder(TextureColorPalette colorPalette)
   {
      this.colorPalette = colorPalette;
   }

   public void changeColorPalette(TextureColorPalette newColorPalette)
   {
      colorPalette = newColorPalette;
   }

   public void clear()
   {
      meshBuilder.clear();
   }

   public void addMesh(MeshDataHolder meshDataHolder, Color color)
   {
      meshBuilder.addMesh(setColor(meshDataHolder, color));
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset, Color color)
   {
      meshBuilder.addMesh(setColor(meshDataHolder, color), offset);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset, AxisAngle4d orientation, Color color)
   {
      meshBuilder.addMesh(setColor(meshDataHolder, color), offset, orientation);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3f offset, Color color)
   {
      meshBuilder.addMesh(setColor(meshDataHolder, color), offset);
   }

   public void addPolyon(List<Point3d> polygon, Color color)
   {
      addMesh(MeshDataGenerator.Polygon(polygon), color);
   }

   public void addBox(float lx, float ly, float lz, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), color);
   }

   public void addBox(float lx, float ly, float lz, Tuple3f offset, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset, color);
   }

   public void addBox(double lx, double ly, double lz, Tuple3d offset, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset, color);
   }

   public void addCube(float size, Color color)
   {
      addBox(size, size, size, color);
   }

   public void addCube(float size, Tuple3f pointsOffset, Color color)
   {
      addBox(size, size, size, pointsOffset, color);
   }

   public void addCube(double size, Tuple3d pointsOffset, Color color)
   {
      addBox(size, size, size, pointsOffset, color);
   }

   public void addCube(double size, double xOffset, double yOffset, double zOffset, Color color)
   {
      addBox(size, size, size, new Point3d(xOffset, yOffset, zOffset), color);
   }

   public void addCylinder(double height, double radius, Tuple3d offset, Color color)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, color);
   }

   public void addCylinder(double height, double radius, Tuple3d offset, AxisAngle4d orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, orientation, color);
   }

   public void addCone(double height, double radius, Tuple3d offset, Color color)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, color);
   }

   public void addCone(double height, double radius, Tuple3d offset, AxisAngle4d orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, orientation, color);
   }

   public void addLine(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth, Color color)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth), color);
   }

   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth, Color color)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth), color);
   }

   public void addLine(Point3d start, Point3d end, double lineWidth, Color color)
   {
      addMesh(MeshDataGenerator.Line(start, end, lineWidth), color);
   }

   public void addMultiLine(Point3d[] points, double lineWidth, Color color, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3d start = points[i - 1];
         Point3d end = points[i];
         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3d start = points[points.length - 1];
         Point3d end = points[0];
         addLine(start, end, lineWidth, color);
      }
   }

   public void addMultiLine(List<Point3d> points, double lineWidth, Color color, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3d start = points.get(i - 1);
         Point3d end = points.get(i);
         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3d start = points.get(points.size() - 1);
         Point3d end = points.get(0);
         addLine(start, end, lineWidth, color);
      }
   }

   private MeshDataHolder setColor(MeshDataHolder input, Color color)
   {
      if (input == null)
         return null;
      Point3f[] vertices = input.getVertices();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3f[] vertexNormals = input.getVertexNormals();
      TexCoord2f[] inputTexturePoints = input.getTexturePoints();
      TexCoord2f[] outputTexturePoints = new TexCoord2f[inputTexturePoints.length];
      float[] textureLocation = colorPalette.getTextureLocation(color);
      for (int i = 0; i < inputTexturePoints.length; i++)
         outputTexturePoints[i] = new TexCoord2f(textureLocation);
      return new MeshDataHolder(vertices, outputTexturePoints, triangleIndices, vertexNormals);
   }

   public MeshDataHolder generateMeshDataHolder()
   {
      return meshBuilder.generateMeshDataHolder();
   }

   public Mesh generateMesh()
   {
      return meshBuilder.generateMesh();
   }

   public Material generateMaterial()
   {
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseMap(colorPalette.getColorPalette());
      return material;
   }
}
