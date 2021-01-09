package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
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
public class GDXMultiColorMeshBuilder
{
   private static final int DEFAULT_RES = 32;

   private int hueResolution = 256;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   GDXMeshBuilder meshBuilder = new GDXMeshBuilder();

   public GDXMultiColorMeshBuilder()
   {
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

   public float[] getTextureLocation(Color color)
   {
      float x;
      float[] hsv = new float[3];
      color.toHsv(hsv);
      x = (float) (hsv[0] / 360.0);
      float y = 0.5f;
      return new float[] {x, y};
   }

   public Mesh generateMesh()
   {
      return meshBuilder.generateMesh();
   }

//   public Material generateMaterial(AssetManager assetManager)
//   {
//      Material material = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
//      Texture texture = assetManager.loadTexture("palette.png");
//      material.setTexture("DiffuseMap", texture);
//      material.createMeshDataWithColor("Diffuse", ColorRGBA.White);
//      return material;
//   }
}