package us.ihmc.graphics3DDescription;

import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

public class MeshDataBuilder
{
   private static final int DEFAULT_RES = 32;

   private final ModifiableMeshDataHolder meshDataHolder = new ModifiableMeshDataHolder();

   public MeshDataBuilder()
   {
      clear();
   }

   public void clear()
   {
      meshDataHolder.clear();
   }

   public void addMesh(MeshDataBuilder other)
   {
      meshDataHolder.add(other.meshDataHolder, true);
   }

   public void addMesh(MeshDataHolder meshDataHolder)
   {
      this.meshDataHolder.add(meshDataHolder, true);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset)
   {
      addMesh(MeshDataHolder.translate(meshDataHolder, offset));
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(MeshDataHolder.rotate(meshDataHolder, orientation), offset);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3f offset)
   {
      addMesh(MeshDataHolder.translate(meshDataHolder, offset));
   }

   public void addPointCloud(Point3f[] points, float radius)
   {
      for (Point3f point : points)
         addSphere(radius, point);
   }

   public void addSphere(float radius)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES));
   }

   public void addSphere(float radius, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), offset);
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

   public void addCube(double size, double xOffset, double yOffset, double zOffset)
   {
      addBox(size, size, size, new Point3d(xOffset, yOffset, zOffset));
   }

   public void addBox(float lx, float ly, float lz)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null));
   }

   public void addBox(float lx, float ly, float lz, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset);
   }

   public void addBox(double lx, double ly, double lz, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset);
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

   public MeshDataHolder generateMeshDataHolder()
   {
      return meshDataHolder.createMeshDataHolder();
   }
}
