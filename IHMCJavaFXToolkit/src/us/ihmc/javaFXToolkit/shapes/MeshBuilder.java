package us.ihmc.javaFXToolkit.shapes;

import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import javafx.scene.shape.Mesh;
import us.ihmc.graphics3DAdapter.graphics.MeshDataGenerator;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

public class MeshBuilder
{
   static final int DEFAULT_RES = 32;

   private MeshDataHolder meshDataHolder;

   public MeshBuilder()
   {
      clear();
   }

   public void clear()
   {
      meshDataHolder = null;
   }

   public void addMesh(MeshBuilder other)
   {
      addMesh(other.meshDataHolder);
   }

   public void addMesh(MeshDataHolder meshDataHolder)
   {
      if (this.meshDataHolder == null)
         this.meshDataHolder = meshDataHolder;
      else
         this.meshDataHolder = MeshDataHolder.combine(this.meshDataHolder, meshDataHolder, true);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset)
   {
      addMesh(translate(meshDataHolder, offset));
   }
   
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(rotate(meshDataHolder, orientation), offset);
   }
   
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3f offset)
   {
      addMesh(translate(meshDataHolder, offset));
   }

   public void addPolygon(List<Point3d> polygon)
   {
      addMesh(MeshDataGenerator.Polygon(polygon));
   }
   
   public void addCube(float size, Tuple3f cubeOffset)
   {
      addBox(size, size, size, cubeOffset);
   }

   public void addCube(double size, Tuple3d cubeOffset)
   {
      addBox(size, size, size, cubeOffset);
   }

   public void addBox(float lx, float ly, float lz)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true));
   }

   public void addBox(float lx, float ly, float lz, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true), offset);
   }

   public void addBox(double lx, double ly, double lz, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true), offset);
   }

   public void addCylinder(double height, double radius, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset);
   }

   public void addCylinder(double height, double radius, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, orientation);
   }

   public void addCylinder(float height, float radius, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset);
   }

   public void addCone(double height, double radius, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cone(radius, height, DEFAULT_RES), offset);
   }

   public void addCone(double height, double radius, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, orientation);
   }

   public void addCone(float height, float radius, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cone(radius, height, DEFAULT_RES), offset);
   }

   public void addLine(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth));
   }

   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth));
   }

   public void addLine(Point3d start, Point3d end, double lineWidth)
   {
      addMesh(MeshDataGenerator.Line(start, end, lineWidth));
   }

   public void addMultiLine(Point3d[] points, double lineWidth, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3d start = points[i - 1];
         Point3d end = points[i];
         addLine(start, end, lineWidth);
      }

      if (close)
      {
         Point3d start = points[points.length - 1];
         Point3d end = points[0];
         addLine(start, end, lineWidth);
      }
   }

   public void addMultiLine(List<Point3d> points, double lineWidth, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3d start = points.get(i - 1);
         Point3d end = points.get(i);
         addLine(start, end, lineWidth);
      }

      if (close)
      {
         Point3d start = points.get(points.size() - 1);
         Point3d end = points.get(0);
         addLine(start, end, lineWidth);
      }
   }

   public static MeshDataHolder translate(MeshDataHolder input, Tuple3d offset)
   {
      return translate(input, (float) offset.getX(), (float) offset.getY(), (float) offset.getZ());
   }
   
   public static MeshDataHolder translate(MeshDataHolder input, Tuple3f offset)
   {
      return translate(input, offset.getX(), offset.getY(), offset.getZ());
   }

   public static MeshDataHolder translate(MeshDataHolder input, float offsetX, float offsetY, float offsetZ)
   {
      Point3f[] inputVertices = input.getVertices();
      TexCoord2f[] texturePoints = input.getTexturePoints();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3f[] normals = input.getVertexNormals();

      Point3f[] outputVertices = new Point3f[inputVertices.length];
      for (int i = 0; i < inputVertices.length; i++)
      {
         outputVertices[i] = new Point3f(offsetX, offsetY, offsetZ);
         outputVertices[i].add(inputVertices[i]);
      }
      return new MeshDataHolder(outputVertices, texturePoints, triangleIndices, normals);
   }

   public static MeshDataHolder rotate(MeshDataHolder input, AxisAngle4d axisAngle)
   {
      Matrix3f matrix = new Matrix3f();
      matrix.set(axisAngle);
      return rotate(input, matrix);
   }

   public static MeshDataHolder rotate(MeshDataHolder input, Matrix3d matrix)
   {
      return rotate(input, new Matrix3f(matrix));
   }

   public static MeshDataHolder rotate(MeshDataHolder input, Matrix3f matrix)
   {
      TexCoord2f[] texturePoints = input.getTexturePoints();
      int[] triangleIndices = input.getTriangleIndices();
      Point3f[] inputVertices = input.getVertices();
      Vector3f[] inputNormals = input.getVertexNormals();

      Point3f[] outputVertices = new Point3f[inputVertices.length];
      Vector3f[] outputNormals = new Vector3f[inputNormals.length];

      for (int i = 0; i < inputVertices.length; i++)
      {
         outputVertices[i] = new Point3f();
         outputNormals[i] = new Vector3f();
         matrix.transform(inputVertices[i], outputVertices[i]);
         matrix.transform(inputNormals[i], outputNormals[i]);
      }
      return new MeshDataHolder(outputVertices, texturePoints, triangleIndices, outputNormals);
   }

   public Mesh generateMesh()
   {
      return JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder);
   }
}
