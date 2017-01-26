package us.ihmc.graphicsDescription;

import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import us.ihmc.robotics.geometry.RigidBodyTransform;


/**
 * This class provides a simple way of combining several shapes/meshes into one mesh that can be then used in JavaFX or JME using the corresponding mesh data interpreter.
 * It can be used as a one-time tool or can be recycled by using the method {@link #clear()} every time a new mesh is to be created.
 * @author Sylvain Bertrand
 */
public class MeshDataBuilder
{
   private static final int DEFAULT_RES = 32;

   private final ModifiableMeshDataHolder meshDataHolder = new ModifiableMeshDataHolder();

   /**
    * Creates an empty builder.
    */
   public MeshDataBuilder()
   {
      clear();
   }

   /**
    * Add a box to this builder.
    * @param lx box length along the x-axis.
    * @param ly box length along the y-axis.
    * @param lz box length along the z-axis.
    * @param offset coordinate of the box center. Not modified.
    */
   public void addBox(double lx, double ly, double lz, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset);
   }

   /**
    * Add a box centered at (0, 0, 0) to this builder.
    * @param lx box length along the x-axis.
    * @param ly box length along the y-axis.
    * @param lz box length along the z-axis.
    */
   public void addBox(float lx, float ly, float lz)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null));
   }

   /**
    * Add a box to this builder.
    * @param lx box length along the x-axis.
    * @param ly box length along the y-axis.
    * @param lz box length along the z-axis.
    * @param offset coordinate of the box center. Not modified.
    */
   public void addBox(float lx, float ly, float lz, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with the highest z value.
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param offset coordinate of the cone's base center. Not modified.
    */
   public void addCone(double height, double radius, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cone(radius, height, DEFAULT_RES), offset);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with the highest z value in its local coordinate system.
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param offset coordinate of the cone's base center. Not modified.
    * @param orientation axis-angle describing the cone orientation with respect to world. Not modified.
    */
   public void addCone(double height, double radius, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, orientation);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with the highest z value.
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param offset coordinate of the cone's base center. Not modified.
    */
   public void addCone(float height, float radius, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cone(radius, height, DEFAULT_RES), offset);
   }

   /**
    * Add a cube to this builder.
    * @param size edge length of the cube.
    * @param xOffset x-coordinate of the cube's center.
    * @param yOffset y-coordinate of the cube's center.
    * @param zOffset z-coordinate of the cube's center.
    */
   public void addCube(double size, double xOffset, double yOffset, double zOffset)
   {
      addBox(size, size, size, new Point3d(xOffset, yOffset, zOffset));
   }

   /**
    * Add a cube to this builder.
    * @param size edge length of the cube.
    * @param cubeOffset coordinates of the cube's center. Not modified.
    */
   public void addCube(double size, Tuple3d cubeOffset)
   {
      addBox(size, size, size, cubeOffset);
   }

   /**
    * Add a cube to this builder.
    * @param size edge length of the cube.
    * @param cubeOffset coordinates of the cube's center. Not modified.
    */
   public void addCube(float size, Tuple3f cubeOffset)
   {
      addBox(size, size, size, cubeOffset);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    * @param height height along z of the cylinder.
    * @param radius the cylinder's radius.
    * @param offset coordinates of the cylinder's center. Not modified.
    */
   public void addCylinder(double height, double radius, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis in its local coordinate system.
    * @param height height along z of the cylinder.
    * @param radius the cylinder's radius.
    * @param offset coordinates of the cylinder's center. Not modified.
    * @param orientation axis-angle describing the cylinder orientation with respect to world. Not modified.
    */
   public void addCylinder(double height, double radius, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, orientation);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    * @param height height along z of the cylinder.
    * @param radius the cylinder's radius.
    * @param offset coordinates of the cylinder's center. Not modified.
    */
   public void addCylinder(float height, float radius, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset);
   }

   /**
    * Add a 3D line to this builder.
    * @param x0 x-coordinate of the line start.
    * @param y0 y-coordinate of the line start.
    * @param z0 z-coordinate of the line start.
    * @param xf x-coordinate of the line end.
    * @param yf y-coordinate of the line end.
    * @param zf z-coordinate of the line end.
    * @param lineWidth width of the line.
    */
   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth));
   }

   /**
    * Add a 3D line to this builder.
    * @param x0 x-coordinate of the line start.
    * @param y0 y-coordinate of the line start.
    * @param z0 z-coordinate of the line start.
    * @param xf x-coordinate of the line end.
    * @param yf y-coordinate of the line end.
    * @param zf z-coordinate of the line end.
    * @param lineWidth width of the line.
    */
   public void addLine(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth));
   }

   /**
    * Add a 3D line to this builder.
    * @param start start coordinate of the line. Not modified.
    * @param end end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    */
   public void addLine(Tuple3d start, Tuple3d end, double lineWidth)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth);
   }

   /**
    * Add a 3D line to this builder.
    * @param start start coordinate of the line. Not modified.
    * @param end end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    */
   public void addLine(Tuple3f start, Tuple3f end, float lineWidth)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth);
   }

   /**
    * Combines the mesh contained in {@code other} with the mesh contained in this builder.
    * @param other the other builder holding on the mesh to combine. Not Modified.
    */
   public void addMesh(MeshDataBuilder other)
   {
      meshDataHolder.add(other.meshDataHolder, true);
   }

   /**
    * Combines the given mesh with the mesh contained in this builder.
    * @param meshDataHolder the mesh to combine. Not Modified.
    */
   public void addMesh(MeshDataHolder meshDataHolder)
   {
      this.meshDataHolder.add(meshDataHolder, true);
   }

   /**
    * Translates then combines the given mesh with the mesh contained in this builder.
    * @param meshDataHolder the mesh to translate and combine. Not Modified.
    * @param offset the translation to apply to the given mesh. Not modified.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset)
   {
      addMesh(MeshDataHolder.translate(meshDataHolder, offset));
   }

   /**
    * Rotates, translates, then combines the given mesh with the mesh contained in this builder.
    * @param meshDataHolder the mesh to translate and combine. Not Modified.
    * @param offset the translation to apply to the given mesh. Not modified.
    * @param orientation the axis-angle describing the rotation to apply to the given mesh. Not modified.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3d offset, AxisAngle4d orientation)
   {
      addMesh(MeshDataHolder.rotate(meshDataHolder, orientation), offset);
   }

   /**
    * Translates then combines the given mesh with the mesh contained in this builder.
    * @param meshDataHolder the mesh to translate and combine. Not Modified.
    * @param offset the translation to apply to the given mesh. Not modified.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3f offset)
   {
      addMesh(MeshDataHolder.translate(meshDataHolder, offset));
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * @param points coordinates of the line end points. Not modified. 
    * @param lineWidth width of the lines.
    * @param close whether the end of the given array of points should be connected to the beginning or not.
    */
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

   /**
    * Add a series of connected 3D lines to this builder.
    * @param points coordinates of the line end points. Not modified. 
    * @param lineWidth width of the lines.
    * @param close whether the end of the given array of points should be connected to the beginning or not.
    */
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

   /**
    * Add a polygon to this builder, given its 2D vertex coordinates and its transform to world.
    * No check is performed on the ordering of the vertices.
    * @param transformToWorld to polygon's transform to world. Not modified.
    * @param polygon the polygon's 2D vertices. Not modified.
    */
   public void addPolygon(RigidBodyTransform transformToWorld, List<Point2d> polygon)
   {
      addMesh(MeshDataGenerator.Polygon(transformToWorld, polygon));
   }

   /**
    * Add a sphere centered to this builder.
    * @param radius the sphere radius.
    * @param offset the coordinate of the sphere. Not modified.
    */
   public void addSphere(double radius, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), offset);
   }

   /**
    * Add a sphere centered at (0, 0, 0) to this builder.
    * @param radius the sphere radius.
    */
   public void addSphere(float radius)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES));
   }

   /**
    * Add a sphere centered to this builder.
    * @param radius the sphere radius.
    * @param offset the coordinate of the sphere. Not modified.
    */
   public void addSphere(float radius, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), offset);
   }

   /**
    * Add an array of spheres to this builder.
    * @param radius the radius of the spheres. Not modified.
    * @param offsets the coordinates of each sphere. Not modified.
    */
   public void addSpheres(float radius, Point3f[] offsets)
   {
      for (Point3f point : offsets)
         addSphere(radius, point);
   }

   /**
    * Add a regular tetrahedron to this builder.
    * @param edgeLength edge length of the tetrahedron.
    * @param offset coordinates of the center of the tetrahedron's circumscribed sphere. Not modified.
    */
   public void addTetrahedron(double edgeLength, Tuple3d offset)
   {
      addMesh(MeshDataGenerator.Tetrahedron(edgeLength), offset);
   }

   /**
    * Add a regular tetrahedron to this builder.
    * @param edgeLength edge length of the tetrahedron.
    * @param offset coordinates of the center of the tetrahedron's circumscribed sphere. Not modified.
    */
   public void addTetrahedron(float edgeLength, Tuple3f offset)
   {
      addMesh(MeshDataGenerator.Tetrahedron(edgeLength), offset);
   }

   /**
    * Clears the meshes contained in this builder.
    */
   public void clear()
   {
      meshDataHolder.clear();
   }

   /**
    * @return the resulting mesh as an immutable mesh ready to be interpreted by the adequate mesh data interpreter. 
    */
   public MeshDataHolder generateMeshDataHolder()
   {
      return meshDataHolder.createMeshDataHolder();
   }
}
