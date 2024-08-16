package us.ihmc.rdx.mesh;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;

import java.util.Arrays;
import java.util.List;

/**
 * TODO: Make interface and use Color from ihmc-graphics-description
 */
public class RDXMultiColorMeshBuilder
{
   private static final int DEFAULT_RES = 32;
   private static final float TwoPi = 2.0f * (float) Math.PI;
   private static Texture paletteTexture;

   private int hueResolution = 256;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   RDXMeshBuilder meshBuilder = new RDXMeshBuilder();

   public RDXMultiColorMeshBuilder()
   {
   }

   /**
    * Add a box to this builder.
    *
    * @param lx     box length along the x-axis.
    * @param ly     box length along the y-axis.
    * @param lz     box length along the z-axis.
    * @param color  color of the box. Color accuracy depends on the color palette in use.
    */
   public void addBox(double lx, double ly, double lz, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), color);
   }

   /**
    * Add a box to this builder.
    *
    * @param lx     box length along the x-axis.
    * @param ly     box length along the y-axis.
    * @param lz     box length along the z-axis.
    * @param offset coordinate of the box center. Not modified.
    * @param color  color of the box. Color accuracy depends on the color palette in use.
    */
   public void addBox(double lx, double ly, double lz, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset, color);
   }

   /**
    * Add a box centered at (0, 0, 0) to this builder.
    *
    * @param lx    box length along the x-axis.
    * @param ly    box length along the y-axis.
    * @param lz    box length along the z-axis.
    * @param color color of the box. Color accuracy depends on the color palette in use.
    */
   public void addBox(float lx, float ly, float lz, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), color);
   }

   /**
    * Add a box to this builder.
    *
    * @param lx     box length along the x-axis.
    * @param ly     box length along the y-axis.
    * @param lz     box length along the z-axis.
    * @param offset coordinate of the box center. Not modified.
    * @param color  color of the box. Color accuracy depends on the color palette in use.
    */
   public void addBox(float lx, float ly, float lz, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset, color);
   }

   /**
    * Add a box to this builder.
    *
    * @param lx     box length along the x-axis.
    * @param ly     box length along the y-axis.
    * @param lz     box length along the z-axis.
    * @param offset coordinate of the box center. Not modified.
    * @param orientation axis-angle describing the box orientation with respect to world. Not
    *                    modified.
    * @param color  color of the box. Color accuracy depends on the color palette in use.
    */
   public void addBox(float lx, float ly, float lz, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset, orientation, color);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value in its local coordinate system.
    *
    * @param height      height along z of the cone.
    * @param radius      radius of the cone's base.
    * @param offset      coordinate of the cone's base center. Not modified.
    * @param orientation axis-angle describing the cone orientation with respect to world. Not
    *                    modified.
    * @param color       color of the cone. Color accuracy depends on the color palette in use.
    */
   public void addCone(double height, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, orientation, color);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value.
    *
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param offset coordinate of the cone's base center. Not modified.
    * @param color  color of the cone. Color accuracy depends on the color palette in use.
    */
   public void addCone(double height, double radius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, color);
   }

   /**
    * Add a cube to this builder.
    *
    * @param size    edge length of the cube.
    * @param xOffset x-coordinate of the cube's center.
    * @param yOffset y-coordinate of the cube's center.
    * @param zOffset z-coordinate of the cube's center.
    * @param color   color of the cube. Color accuracy depends on the color palette in use.
    */
   public void addCube(double size, double xOffset, double yOffset, double zOffset, Color color)
   {
      addBox(size, size, size, new Point3D(xOffset, yOffset, zOffset), color);
   }

   /**
    * Add a cube to this builder.
    *
    * @param size       edge length of the cube.
    * @param pointsOffset coordinates of the cube's center. Not modified.
    * @param color      color of the cube. Color accuracy depends on the color palette in use.
    */
   public void addCube(double size, Tuple3DReadOnly pointsOffset, Color color)
   {
      addBox(size, size, size, pointsOffset, color);
   }

   /**
    * Add a cube to this builder.
    *
    * @param size       edge length of the cube.
    * @param pointsOffset coordinates of the cube's center. Not modified.
    * @param color      color of the cube. Color accuracy depends on the color palette in use.
    */
   public void addCube(float size, Tuple3DReadOnly pointsOffset, Color color)
   {
      addBox(size, size, size, pointsOffset, color);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis in its local coordinate
    * system.
    *
    * @param height      height along z of the cylinder.
    * @param radius      the cylinder's radius.
    * @param offset      coordinates of the cylinder's center. Not modified.
    * @param orientation axis-angle describing the cylinder orientation with respect to world. Not
    *                    modified.
    * @param color       color of the cylinder. Color accuracy depends on the color palette in use.
    */
   public void addCylinder(double height, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, orientation, color);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    *
    * @param height height along z of the cylinder.
    * @param radius the cylinder's radius.
    * @param offset coordinates of the cylinder's center. Not modified.
    * @param color  color of the cylinder. Color accuracy depends on the color palette in use.
    */
   public void addCylinder(double height, double radius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, color);
   }

   /**
    * Add a hollow cylinder to this builder. Its axis is aligned with the z-axis in its local coordinate
    * system.
    *
    * @param height      height along z of the cylinder.
    * @param outerRadius the cylinder's outer radius.
    * @param innerRadius the cylinder's inner radius.
    * @param offset      coordinates of the cylinder's center. Not modified.
    * @param orientation axis-angle describing the cylinder orientation with respect to world. Not
    *                    modified.
    * @param color       color of the cylinder. Color accuracy depends on the color palette in use.
    */
   public void addHollowCylinder(double height, double outerRadius, double innerRadius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, Color color)
   {
      addMesh(HollowCylinder(outerRadius, innerRadius, height, DEFAULT_RES), offset, orientation, color);
   }

   /**
    * Add a hollow cylinder to this builder. Its axis is aligned with the z-axis.
    *
    * @param height height along z of the cylinder.
    * @param outerRadius the cylinder's outer radius.
    * @param innerRadius the cylinder's inner radius.
    * @param offset coordinates of the cylinder's center. Not modified.
    * @param color  color of the cylinder. Color accuracy depends on the color palette in use.
    */
   public void addHollowCylinder(double height, double outerRadius, double innerRadius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(HollowCylinder(outerRadius, innerRadius, height, DEFAULT_RES), offset, color);
   }

   public static MeshDataHolder HollowCylinder(double outerRadius, double innerRadius, double height, int N)
   {
      return HollowCylinder((float) outerRadius, (float) innerRadius, (float) height, N);
   }

   public static MeshDataHolder HollowCylinder(float outerRadius, float innerRadius, float height, int N)
   {
      Point3D32[] points = new Point3D32[8 * N];
      Vector3D32[] normals = new Vector3D32[8 * N];
      TexCoord2f[] textPoints = new TexCoord2f[8 * N];

      for (int i = 0; i < N; i++)
      {
         double angle = i * TwoPi / N;
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float vertexX = outerRadius * cosAngle;
         float vertexY = outerRadius * sinAngle;

         // Bottom vertices
         points[i] = new Point3D32(vertexX, vertexY, 0.0f);
         normals[i] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[i] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top vertices
         points[i + N] = new Point3D32(vertexX, vertexY, height);
         normals[i + N] = new Vector3D32(0.0f, 0.0f, 1.0f);
         textPoints[i + N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Outer vertices
         // Bottom
         points[i + 2 * N] = new Point3D32(vertexX, vertexY, 0.0f);
         normals[i + 2 * N] = new Vector3D32(cosAngle, sinAngle, 0.0f);
         textPoints[i + 2 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top
         points[i + 3 * N] = new Point3D32(vertexX, vertexY, height);
         normals[i + 3 * N] = new Vector3D32(cosAngle, sinAngle, 0.0f);
         textPoints[i + 3 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
      }

      for (int i = 0; i < N; i++)
      {
         double angle = i * TwoPi / N;
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float vertexX = innerRadius * cosAngle;
         float vertexY = innerRadius * sinAngle;

         // Bottom vertices
         points[i + 4 * N] = new Point3D32(vertexX, vertexY, 0.0f);
         normals[i + 4 * N] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[i + 4 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top vertices
         points[i + 5 * N] = new Point3D32(vertexX, vertexY, height);
         normals[i + 5 * N] = new Vector3D32(0.0f, 0.0f, 1.0f);
         textPoints[i + 5 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Inner vertices
         // Bottom
         points[i + 6 * N] = new Point3D32(vertexX, vertexY, 0.0f);
         normals[i + 6 * N] = new Vector3D32(-cosAngle, -sinAngle, 0.0f);
         textPoints[i + 6 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top
         points[i + 7 * N] = new Point3D32(vertexX, vertexY, height);
         normals[i + 7 * N] = new Vector3D32(-cosAngle, -sinAngle, 0.0f);
         textPoints[i + 7 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
      }

      int numberOfTriangles = 4 * N;
      int[] triangleIndices = new int[6 * numberOfTriangles];

      int index = 0;

      // counter clockwise winding order
      for (int i = 0; i < N; i++)
      { // The bottom cap
         triangleIndices[index++] = i;
         triangleIndices[index++] = i + 4 * N;
         triangleIndices[index++] = (i + 1) % N;
         triangleIndices[index++] = i + 4 * N;
         triangleIndices[index++] = (i + 1) % N + 4 * N;
         triangleIndices[index++] = (i + 1) % N;
      }

      for (int i = 0; i < N; i++)
      { // The top cap
         triangleIndices[index++] = i + N;
         triangleIndices[index++] = (i + 1) % N + N;
         triangleIndices[index++] = i + 5 * N;
         triangleIndices[index++] = i + 5 * N;
         triangleIndices[index++] = (i + 1) % N + N;
         triangleIndices[index++] = (i + 1) % N + 5 * N;
      }

      for (int i = 0; i < N; i++)
      { // The outer cylinder part
         // Lower triangle
         triangleIndices[index++] = i + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = i + 3 * N;
         // Upper triangle
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 3 * N;
         triangleIndices[index++] = i + 3 * N;
      }

      for (int i = 0; i < N; i++)
      { // The inner cylinder part
         // Lower triangle
         triangleIndices[index++] = i + 6 * N;
         triangleIndices[index++] = i + 7 * N;
         triangleIndices[index++] = (i + 1) % N + 6 * N;
         // Upper triangle
         triangleIndices[index++] = (i + 1) % N + 6 * N;
         triangleIndices[index++] = i + 7 * N;
         triangleIndices[index++] = (i + 1) % N + 7 * N;
      }

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param x0        x-coordinate of the line start.
    * @param y0        y-coordinate of the line start.
    * @param z0        z-coordinate of the line start.
    * @param xf        x-coordinate of the line end.
    * @param yf        y-coordinate of the line end.
    * @param zf        z-coordinate of the line end.
    * @param lineWidth width of the line.
    * @param color     color of the line. Color accuracy depends on the color palette in use.
    */
   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth, Color color)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth), color);
   }

   /**
    * Add a 3D line with a color gradient to this builder.
    *
    * @param x0         x-coordinate of the line start.
    * @param y0         y-coordinate of the line start.
    * @param z0         z-coordinate of the line start.
    * @param xf         x-coordinate of the line end.
    * @param yf         y-coordinate of the line end.
    * @param zf         z-coordinate of the line end.
    * @param lineWidth  width of the line.
    * @param startColor color at the line start. Color accuracy depends on the color palette in use.
    * @param endColor   color at the line end. Color accuracy depends on the color palette in use.
    */
   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth, Color startColor, Color endColor)
   {
      addLine((float) x0, (float) y0, (float) z0, (float) xf, (float) yf, (float) zf, (float) lineWidth, startColor, endColor);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param x0        x-coordinate of the line start.
    * @param y0        y-coordinate of the line start.
    * @param z0        z-coordinate of the line start.
    * @param xf        x-coordinate of the line end.
    * @param yf        y-coordinate of the line end.
    * @param zf        z-coordinate of the line end.
    * @param lineWidth width of the line.
    * @param color     color of the line. Color accuracy depends on the color palette in use.
    */
   public void addLine(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth, Color color)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth), color);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param x0         x-coordinate of the line start.
    * @param y0         y-coordinate of the line start.
    * @param z0         z-coordinate of the line start.
    * @param xf         x-coordinate of the line end.
    * @param yf         y-coordinate of the line end.
    * @param zf         z-coordinate of the line end.
    * @param lineWidth  width of the line.
    * @param startColor color at the line start. Color accuracy depends on the color palette in use.
    * @param endColor   color at the line end. Color accuracy depends on the color palette in use.
    */
   public void addLine(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth, Color startColor, Color endColor)
   {
      MeshDataHolder lineMeshData = MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth);

      Point3D32[] vertices = lineMeshData.getVertices();
      TexCoord2f[] texturePoints = lineMeshData.getTexturePoints();

      Point3D32 start = new Point3D32(x0, y0, z0);
      Point3D32 end = new Point3D32(xf, yf, zf);

      for (int i = 0; i < vertices.length; i++)
      {
         if (vertices[i].distanceSquared(start) < vertices[i].distanceSquared(end))
            texturePoints[i].set(getTextureLocation(startColor));
         else
            texturePoints[i].set(getTextureLocation(endColor));
      }

      meshBuilder.addMesh(lineMeshData);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param start     start coordinate of the line. Not modified.
    * @param end       end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    * @param color     color of the line. Color accuracy depends on the color palette in use.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, double lineWidth, Color color)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth, color);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param start      start coordinate of the line. Not modified.
    * @param end        end coordinate of the line. Not modified.
    * @param lineWidth  width of the line.
    * @param startColor color at the line start. Color accuracy depends on the color palette in use.
    * @param endColor   color at the line end. Color accuracy depends on the color palette in use.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, double lineWidth, Color startColor, Color endColor)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth, startColor, endColor);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param start     start coordinate of the line. Not modified.
    * @param end       end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    * @param color     color of the line. Color accuracy depends on the color palette in use.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, float lineWidth, Color color)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth, color);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param start      start coordinate of the line. Not modified.
    * @param end        end coordinate of the line. Not modified.
    * @param lineWidth  width of the line.
    * @param startColor color at the line start. Color accuracy depends on the color palette in use.
    * @param endColor   color at the line end. Color accuracy depends on the color palette in use.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, float lineWidth, Color startColor, Color endColor)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth, startColor, endColor);
   }

   public void addCapsule(double height, double xRadius, double yRadius, double zRadius, int latitudeN, int longitudeN, Color color)
   {
      addCapsule((float) height, (float) xRadius, (float) yRadius, (float) zRadius, latitudeN, longitudeN, color);
   }

   public void addCapsule(float height, float xRadius, float yRadius, float zRadius, int latitudeN, int longitudeN, Color color)
   {
      addMesh(MeshDataGenerator.Capsule(height, xRadius, yRadius, zRadius, latitudeN, longitudeN), color);
   }

   /**
    * Combines the given mesh with the mesh contained in this builder while specifying the color of the
    * given mesh.
    *
    * @param meshDataHolder the mesh to combine. Not modified.
    * @param color          color of the given mesh. Color accuracy depends on the color palette in
    *                       use.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Color color)
   {
      meshBuilder.addMesh(createMeshDataWithColor(meshDataHolder, color));
   }

   /**
    * Rotates, translates, then combines the given mesh with the mesh contained in this builder.
    *
    * @param meshDataHolder the mesh to translate and combine. Not Modified.
    * @param offset         the translation to apply to the given mesh. Not modified.
    * @param orientation    the axis-angle describing the rotation to apply to the given mesh. Not
    *                       modified.
    * @param color          color of the given mesh. Color accuracy depends on the color palette in
    *                       use.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, Color color)
   {
      meshBuilder.addMesh(createMeshDataWithColor(meshDataHolder, color), offset, orientation);
   }

   /**
    * Translates then combines the given mesh with the mesh contained in this builder.
    *
    * @param meshDataHolder the mesh to translate and combine. Not Modified.
    * @param offset         the translation to apply to the given mesh. Not modified.
    * @param color          color of the given mesh. Color accuracy depends on the color palette in
    *                       use.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Tuple3DReadOnly offset, Color color)
   {
      meshBuilder.addMesh(createMeshDataWithColor(meshDataHolder, color), offset);
   }

   /**
    * Add a series of connected 3D lines to this builder.
    *
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param color     color of the multi-line. Color accuracy depends on the color palette in use.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    */
   public void addMultiLine(List<? extends Point3DReadOnly> points, double lineWidth, Color color, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3DReadOnly start = points.get(i - 1);
         Point3DReadOnly end = points.get(i);
         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3DReadOnly start = points.get(points.size() - 1);
         Point3DReadOnly end = points.get(0);
         addLine(start, end, lineWidth, color);
      }
   }

   /**
    * Accepts vertices to draw a box in the order given by Euclid Box3D.
    *
    * @param eightVertices
    * @param lineWidth
    * @param color
    */
   public void addMultiLineBox(Point3DReadOnly[] eightVertices, double lineWidth, Color color)
   {
      if (eightVertices.length != 8)
         throw new RuntimeException("There should be 8 vertices in this array");

      addLine(eightVertices[0], eightVertices[1], lineWidth, color); // x+y+z+  draw top
      addLine(eightVertices[1], eightVertices[3], lineWidth, color); // x-y-z+
      addLine(eightVertices[3], eightVertices[2], lineWidth, color); // x-y+z+
      addLine(eightVertices[2], eightVertices[0], lineWidth, color); // x+y-z+
      addLine(eightVertices[0], eightVertices[4], lineWidth, color); // x+y+z+
      addLine(eightVertices[4], eightVertices[5], lineWidth, color); // x+y+z-  go down
      addLine(eightVertices[5], eightVertices[1], lineWidth, color); // x-y-z-  leg 1
      addLine(eightVertices[1], eightVertices[5], lineWidth, color); // x-y-z+
      addLine(eightVertices[5], eightVertices[7], lineWidth, color); // x-y-z-
      addLine(eightVertices[7], eightVertices[3], lineWidth, color); // x-y+z-  leg 2
      addLine(eightVertices[3], eightVertices[7], lineWidth, color); // x-y+z+
      addLine(eightVertices[7], eightVertices[6], lineWidth, color); // x-y+z-
      addLine(eightVertices[6], eightVertices[2], lineWidth, color); // x+y-z-  leg 3
      addLine(eightVertices[2], eightVertices[6], lineWidth, color); // x+y-z+
      addLine(eightVertices[6], eightVertices[4], lineWidth, color); // x+y-z-
   }

   /**
    * Add a series of connected 3D lines to this builder.
    *
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param color     color of the multi-line. Color accuracy depends on the color palette in use.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    */
   public void addMultiLine(Point3DReadOnly[] points, double lineWidth, Color color, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3DReadOnly start = points[i - 1];
         Point3DReadOnly end = points[i];
         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3DReadOnly start = points[points.length - 1];
         Point3DReadOnly end = points[0];
         addLine(start, end, lineWidth, color);
      }
   }

   /**
    * Add a series of connected 2D lines to this builder.
    *
    * @param transformToWorld the transform from the mult-line local coordinates to world. Not
    *                         modified.
    * @param points           coordinates of the line end points. Not modified.
    * @param lineWidth        width of the lines.
    * @param color            color of the multi-line. Color accuracy depends on the color palette in
    *                         use.
    * @param close            whether the end of the given array of points should be connected to the
    *                         beginning or not.
    */
   public void addMultiLine(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DReadOnly> points, double lineWidth, Color color, boolean close)
   {
      if (points.size() < 2)
         return;

      Point3D start = new Point3D();
      Point3D end = new Point3D();

      for (int i = 1; i < points.size(); i++)
      {
         Point2DReadOnly start2d = points.get(i - 1);
         Point2DReadOnly end2d = points.get(i);

         start.set(start2d.getX(), start2d.getY(), 0.0);
         end.set(end2d.getX(), end2d.getY(), 0.0);
         transformToWorld.transform(start);
         transformToWorld.transform(end);

         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point2DReadOnly start2d = points.get(points.size() - 1);
         Point2DReadOnly end2d = points.get(0);

         start.set(start2d.getX(), start2d.getY(), 0.0);
         end.set(end2d.getX(), end2d.getY(), 0.0);
         transformToWorld.transform(start);
         transformToWorld.transform(end);

         addLine(start, end, lineWidth, color);
      }
   }

   /**
    * Add a series of connected 2D lines to this builder.
    *
    * @param transformToWorld the transform from the mult-line local coordinates to world. Not
    *                         modified.
    * @param points           coordinates of the line end points. Not modified.
    * @param lineWidth        width of the lines.
    * @param color            color of the multi-line. Color accuracy depends on the color palette in
    *                         use.
    * @param close            whether the end of the given array of points should be connected to the
    *                         beginning or not.
    */
   public void addMultiLine(RigidBodyTransformReadOnly transformToWorld, Point2DReadOnly[] points, double lineWidth, Color color, boolean close)
   {
      addMultiLine(transformToWorld, Arrays.asList(points), lineWidth, color, close);
   }

   /**
    * Add a 2D polygon to this builder.
    *
    * @param transformToWorld the transform from the polygon's local coordinates to world. Not
    *                         modified.
    * @param polygon          the polygon to render.
    * @param color            color of the polygon. Color accuracy depends on the color palette in use.
    */
   public void addPolygon(RigidBodyTransformReadOnly transformToWorld, ConvexPolygon2DReadOnly polygon, Color color)
   {
      addMesh(MeshDataGenerator.Polygon(transformToWorld, polygon), color);
   }

   /**
    * Add a polygon to this builder. No sanity check is performed on the polygon's vertices.
    *
    * @param polygon the polygon 3D vertices.
    * @param color   color of the polygon. Color accuracy depends on the color palette in use.
    */
   public void addPolygon(List<? extends Point3DReadOnly> polygon, Color color)
   {
      addMesh(MeshDataGenerator.Polygon(polygon), color);
   }

   /**
    * Add a polygon to this builder. No sanity check is performed on the polygon's vertices.
    *
    * @param transformToWorld the transform from the polygon's local coordinates to world. Not
    *                         modified.
    * @param polygon the polygon 3D vertices.
    * @param color   color of the polygon. Color accuracy depends on the color palette in use.
    */
   public void addPolygon(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DReadOnly> polygon, Color color)
   {
      addMesh(MeshDataGenerator.Polygon(transformToWorld, polygon), color);
   }


   /**
    * Add a sphere centered to this builder.
    *
    * @param radius the sphere radius.
    * @param offset the coordinate of the sphere. Not modified.
    * @param color  color of the sphere. Color accuracy depends on the color palette in use.
    */
   public void addSphere(double radius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), offset, color);
   }

   /**
    * Add a sphere centered at (0, 0, 0) to this builder.
    *
    * @param radius the sphere radius.
    * @param color  color of the sphere. Color accuracy depends on the color palette in use.
    */
   public void addSphere(float radius, Color color)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), color);
   }

   /**
    * Add a sphere centered to this builder.
    *
    * @param radius the sphere radius.
    * @param offset the coordinate of the sphere. Not modified.
    * @param color  color of the sphere. Color accuracy depends on the color palette in use.
    */
   public void addSphere(float radius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), offset, color);
   }

   /**
    * Add an ellipsoid centered to this builder.
    *
    * @param xRadius the x radius
    * @param yRadius the y radius
    * @param zRadius ths z radius
    * @param offset the coordinate of the ellipsoid
    * @param color color of the ellipsoid. Color accuracy depends on the color palette in use.
    */
   public void addEllipsoid(double xRadius, double yRadius, double zRadius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Ellipsoid(xRadius, yRadius, zRadius, DEFAULT_RES, DEFAULT_RES), offset, color);
   }

   /**
    * Add an hemi-ellipsoid centered to this builder.
    *
    * @param xRadius the x radius
    * @param yRadius the y radius
    * @param zRadius ths z radius
    * @param offset the coordinate of the hemi-ellipsoid
    * @param color color of the hemi-ellipsoid. Color accuracy depends on the color palette in use.
    */
   public void addHemiEllipsoid(double xRadius, double yRadius, double zRadius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.HemiEllipsoid(xRadius, yRadius, zRadius, DEFAULT_RES, DEFAULT_RES), offset, color);
   }

   /**
    * Add a regular tetrahedron to this builder.
    *
    * @param edgeLength edge length of the tetrahedron.
    * @param offset     coordinates of the center of the tetrahedron's circumscribed sphere. Not
    *                   modified.
    * @param color      color of the tetrahedron. Color accuracy depends on the color palette in use.
    */
   public void addTetrahedron(double edgeLength, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Tetrahedron(edgeLength), offset, color);
   }

   /**
    * Add a regular tetrahedron to this builder.
    *
    * @param edgeLength edge length of the tetrahedron.
    * @param offset     coordinates of the center of the tetrahedron's circumscribed sphere. Not
    *                   modified.
    * @param color      color of the tetrahedron. Color accuracy depends on the color palette in use.
    */
   public void addTetrahedron(float edgeLength, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Tetrahedron(edgeLength), offset, color);
   }

   public void addWedge(double lx, double ly, double lz, Color color)
   {
      addMesh(MeshDataGenerator.Wedge(lx, ly, lz), color);
   }

   /**
    * Add a isosceles triangular prism to this builder. The prism's origin is at the center of it's base face with the peak on top
    * like a roofline along the Y axis. The depth of the prism is along the Y axis.
    *
    * @param triangleWidth   The width/base of the upward pointing isosceles triangle
    * @param triangleHeight  The height of the upward pointing isosceles triangle
    * @param prismThickness  The thichness/depth extruded by the triangle to make a prism
    * @param offset          Coordinates of the origin
    */
   public void addIsoscelesTriangularPrism(double triangleWidth, double triangleHeight, double prismThickness, Tuple3DReadOnly offset, Color color)
   {
      addMesh(IsoscelesTriangularPrism(triangleWidth, triangleHeight, prismThickness), offset, color);
   }

   /**
    * Add a isosceles triangular prism to this builder. The prism's origin is at the center of it's base face with the peak on top
    * like a roofline along the Y axis. The depth of the prism is along the Y axis.
    *
    * @param triangleWidth   The width/base of the upward pointing isosceles triangle
    * @param triangleHeight  The height of the upward pointing isosceles triangle
    * @param prismThickness  The thichness/depth extruded by the triangle to make a prism
    * @param offset          Coordinates of the origin
    * @param orientation     Orientation of the origin
    */
   public void addIsoscelesTriangularPrism(double triangleWidth,
                                           double triangleHeight,
                                           double prismThickness,
                                           Tuple3DReadOnly offset,
                                           Orientation3DReadOnly orientation,
                                           Color color)
   {
      addMesh(IsoscelesTriangularPrism(triangleWidth, triangleHeight, prismThickness), offset, orientation, color);
   }

   public static MeshDataHolder IsoscelesTriangularPrism(double triangleWidth, double triangleHeight, double prismThickness)
   {
      return IsoscelesTriangularPrism((float) triangleWidth, (float) triangleHeight, (float) prismThickness);
   }

   public static MeshDataHolder IsoscelesTriangularPrism(float triangleWidth, float triangleHeight, float prismThickness)
   {
      Point3D32[] points = new Point3D32[18];
      Vector3D32[] normals = new Vector3D32[18];
      TexCoord2f[] textPoints = new TexCoord2f[18];

      // Bottom face vertices
      points[0] = new Point3D32(-triangleWidth / 2.0f, -prismThickness / 2.0f, 0.0f);
      normals[0] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      points[1] = new Point3D32(triangleWidth / 2.0f, -prismThickness / 2.0f, 0.0f);
      normals[1] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      points[2] = new Point3D32(triangleWidth / 2.0f, prismThickness / 2.0f, 0.0f);
      normals[2] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      points[3] = new Point3D32(-triangleWidth / 2.0f, prismThickness / 2.0f, 0.0f);
      normals[3] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      // Back face vertices
      points[4] = new Point3D32(0.0f, -prismThickness / 2.0f, triangleHeight);
      normals[4] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[4] = new TexCoord2f(0.0f, 1.0f);
      points[5] = new Point3D32(0.0f, prismThickness / 2.0f, triangleHeight);
      normals[5] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[5] = new TexCoord2f(1.0f, 1.0f);
      points[6] = new Point3D32(points[2]);
      normals[6] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[6] = new TexCoord2f(textPoints[2]);
      points[7] = new Point3D32(points[1]);
      normals[7] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[7] = new TexCoord2f(textPoints[1]);

      // Top face vertices
      float wedgeAngle = (float) Math.atan2(triangleHeight, triangleWidth / 2.0);
      points[8] = new Point3D32(points[0]);
      normals[8] = new Vector3D32(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[8] = new TexCoord2f(textPoints[0]);
      points[9] = new Point3D32(points[4]);
      normals[9] = new Vector3D32(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[9] = new TexCoord2f(textPoints[4]);
      points[10] = new Point3D32(points[5]);
      normals[10] = new Vector3D32(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[10] = new TexCoord2f(textPoints[5]);
      points[11] = new Point3D32(points[3]);
      normals[11] = new Vector3D32(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[11] = new TexCoord2f(textPoints[3]);

      // Right face vertices
      points[12] = new Point3D32(points[0]);
      normals[12] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[12] = new TexCoord2f(textPoints[0]);
      points[13] = new Point3D32(points[1]);
      normals[13] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[13] = new TexCoord2f(textPoints[1]);
      points[14] = new Point3D32(points[4]);
      normals[14] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[14] = new TexCoord2f(textPoints[4]);

      // Left face vertices
      points[15] = new Point3D32(points[2]);
      normals[15] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[15] = new TexCoord2f(textPoints[2]);
      points[16] = new Point3D32(points[3]);
      normals[16] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[16] = new TexCoord2f(textPoints[3]);
      points[17] = new Point3D32(points[5]);
      normals[17] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[17] = new TexCoord2f(textPoints[5]);

      int numberOfTriangles = 2 * 3 + 2;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Bottom face
      triangleIndices[index++] = 0;
      triangleIndices[index++] = 2;
      triangleIndices[index++] = 1;

      triangleIndices[index++] = 0;
      triangleIndices[index++] = 3;
      triangleIndices[index++] = 2;

      // Back face
      triangleIndices[index++] = 7;
      triangleIndices[index++] = 5;
      triangleIndices[index++] = 4;

      triangleIndices[index++] = 5;
      triangleIndices[index++] = 7;
      triangleIndices[index++] = 6;

      // Top face
      triangleIndices[index++] = 8;
      triangleIndices[index++] = 9;
      triangleIndices[index++] = 10;

      triangleIndices[index++] = 8;
      triangleIndices[index++] = 10;
      triangleIndices[index++] = 11;

      // Right face
      triangleIndices[index++] = 12;
      triangleIndices[index++] = 13;
      triangleIndices[index++] = 14;

      // Left face
      triangleIndices[index++] = 15;
      triangleIndices[index++] = 16;
      triangleIndices[index++] = 17;

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
   }

   public void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, Color color)
   {
      addMesh(MeshDataGenerator.ArcTorus(startAngle, endAngle, majorRadius, minorRadius, 25), color);
   }

   public void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, int resolution, Color color)
   {
      addMesh(MeshDataGenerator.ArcTorus(startAngle, endAngle, majorRadius, minorRadius, resolution), color);
   }

   public void addArrow(double arrowBodyLength, Color color)
   {
      addMesh(MeshDataGeneratorMissing.Arrow(arrowBodyLength), color);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3DReadOnly offset, AxisAngle orientation, Color color)
   {
      meshBuilder.addMesh(createMeshDataWithColor(meshDataHolder, color), offset, orientation);
   }

   private MeshDataHolder createMeshDataWithColor(MeshDataHolder input, Color color)
   {
      if (input == null)
         return null;
      Point3D32[] vertices = input.getVertices();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3D32[] vertexNormals = input.getVertexNormals();
      TexCoord2f[] inputTexturePoints = input.getTexturePoints();
      TexCoord2f[] outputTexturePoints = new TexCoord2f[inputTexturePoints.length];
      float[] textureLocation = getTextureLocation(color);
      for (int i = 0; i < inputTexturePoints.length; i++)
         outputTexturePoints[i] = new TexCoord2f(textureLocation);
      return new MeshDataHolder(vertices, outputTexturePoints, triangleIndices, vertexNormals);
   }

   public static float[] getTextureLocation(Color color)
   {
      // texture 64 vertical pixels of white to black fully saturated hues
      // then, 12 pixels of grayscale blacl left to right white
      // texture is 256 pixels wide to span the different hues red to violet
      // texture is 76 pixels tall
      float[] hsv = new float[3];
      color.toHsv(hsv);

      float hue = hsv[0];
      float saturation = hsv[1];
      float value = hsv[2];
      float x;
      float y;
//      LogTools.info("H: {} S: {} V: {}", hue, saturation, value);

      if (saturation < 0.1) // black and white
      {
         x = value;
         y = 0.98f;
      }
      else
      {
         float percentHeightIsHues = 64.0f / 76.0f;
         x = hue / 360.0f;
         y = percentHeightIsHues - percentHeightIsHues * value / 2.0f;
      }
//      LogTools.info("X: {} Y: {}", x, y);

      // x is 0.0 -> 1.0 is left to right
      // y is 0.0 -> 1.0 top to bottom
      return new float[] {x, y};
   }

   public static String getPalletImagePath()
   {
      return "RGB_24bits_hue_value_palette.png";
   }

   public static Texture loadPaletteTexture()
   {
      if (paletteTexture == null)
         paletteTexture = new Texture(Gdx.files.classpath(getPalletImagePath()));
      return paletteTexture;
   }

   public Mesh generateMesh()
   {
      return meshBuilder.generateMesh();
   }

   public MeshDataHolder generateMeshDataHolder()
   {
      return meshBuilder.generateMeshDataHolder();
   }

   public void clear()
   {
      meshBuilder.clear();
   }
}