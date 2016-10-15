package us.ihmc.graphics3DAdapter.graphics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class MeshDataGenerator
{

   private MeshDataGenerator()
   {
      // Prevent an object being generated.
   }

   public static MeshDataHolder Sphere(double radius, int latitudeN, int longitudeN)
   {
      return Sphere((float) radius, latitudeN, longitudeN);
   }

   public static MeshDataHolder Sphere(float radius, int latitudeN, int longitudeN)
   {
      return Ellipsoid(radius, radius, radius, latitudeN, longitudeN);
   }

   public static MeshDataHolder Ellipsoid(double xRadius, double yRadius, double zRadius, int latitudeN, int longitudeN)
   {
      return Ellipsoid((float) xRadius, (float) yRadius, (float) zRadius, latitudeN, longitudeN);
   }

   public static MeshDataHolder Ellipsoid(float xRadius, float yRadius, float zRaduis, int latitudeN, int longitudeN)
   {
      // Reminder of longitude and latitude: http://www.geographyalltheway.com/ks3_geography/maps_atlases/longitude_latitude.htm
      Point3f points[] = new Point3f[(latitudeN - 1) * longitudeN + 2];
      Vector3f[] normals = new Vector3f[(latitudeN - 1) * longitudeN + 2];
      TexCoord2f textPoints[] = new TexCoord2f[(latitudeN - 1) * longitudeN + 2];

      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         float longitudeAngle = (float) (2.0 * Math.PI * ((float) longitudeIndex / (float) longitudeN));

         for (int latitudeIndex = 1; latitudeIndex < latitudeN; latitudeIndex++)
         {
            float latitudeAngle = (float) (-Math.PI / 2.0f + Math.PI * ((float) latitudeIndex / (float) latitudeN));

            int currentIndex = (latitudeIndex - 1) * longitudeN + longitudeIndex;
            float vertexX = (float) (xRadius * Math.cos(longitudeAngle) * Math.cos(latitudeAngle));
            float vertexY = (float) (yRadius * Math.sin(longitudeAngle) * Math.cos(latitudeAngle));
            float vertexZ = (float) (zRaduis * Math.sin(latitudeAngle));
            points[currentIndex] = new Point3f(vertexX, vertexY, vertexZ);

            float normalX = (float) (Math.cos(longitudeAngle) * Math.cos(latitudeAngle));
            float normalY = (float) (Math.sin(longitudeAngle) * Math.cos(latitudeAngle));
            float normalZ = (float) (Math.sin(latitudeAngle));
            normals[currentIndex] = new Vector3f(normalX, normalY, normalZ);

            float textureX = (float) (longitudeAngle / (2.0 * Math.PI));
            float textureY = (float) (0.5 * Math.sin(latitudeAngle) + 0.5);
            textPoints[currentIndex] = new TexCoord2f(textureX, textureY);
         }
      }

      // South pole
      int southPoleIndex = (latitudeN - 1) * longitudeN;
      points[southPoleIndex] = new Point3f(0.0f, 0.0f, -zRaduis);
      normals[southPoleIndex] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[southPoleIndex] = new TexCoord2f(0.0f, 0.0f);

      // North pole
      int northPoleIndex = (latitudeN - 1) * longitudeN + 1;
      points[northPoleIndex] = new Point3f(0.0f, 0.0f, zRaduis);
      normals[northPoleIndex] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[northPoleIndex] = new TexCoord2f(1.0f, 1.0f);

      int numberOfTriangles = 2 * (latitudeN - 1) * longitudeN + 2 * longitudeN;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Mid-latitude faces
      for (int latitudeIndex = 0; latitudeIndex < latitudeN - 2; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
            int nextLatitudeIndex = (latitudeIndex + 1);

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
         }
      }

      // South pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = southPoleIndex;
         triangleIndices[index++] = nextLongitudeIndex;
         triangleIndices[index++] = longitudeIndex;
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = northPoleIndex;
         triangleIndices[index++] = (latitudeN - 2) * longitudeN + longitudeIndex;
         triangleIndices[index++] = (latitudeN - 2) * longitudeN + nextLongitudeIndex;
      }

      int[] pStripCounts = new int[numberOfTriangles];
      Arrays.fill(pStripCounts, 3);

      return new MeshDataHolder(points, textPoints, triangleIndices, pStripCounts, normals);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b> 
    * @param polygonPoints the vertices of the polygon.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder Polygon(ArrayList<Point3d> polygonPoints)
   {
      return Polygon(polygonPoints, polygonPoints.size());
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b> 
    * @param polygonPoints the vertices of the polygon.
    * @param numberOfVertices will read only the vertices from 0 to numberOfVertices - 1.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder Polygon(ArrayList<Point3d> polygonPoints, int numberOfVertices)
   {
      Point3f[] points = new Point3f[numberOfVertices];
      for (int i = 0; i < numberOfVertices; i++)
      {
         points[i] = new Point3f(polygonPoints.get(i));
      }

      return Polygon(points);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b> 
    * @param polygonPoints the vertices of the polygon.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder Polygon(Point2d[] polygonPoints)
   {
      Point3f[] points = new Point3f[polygonPoints.length];
      for (int i = 0; i < polygonPoints.length; i++)
      {
         Point2d vertex = polygonPoints[i];
         points[i] = new Point3f((float) vertex.getX(), (float) vertex.getY(), 0.0f);
      }
      return Polygon(points);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * @param convexPolygon the polygon to create a mesh from.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder Polygon(ConvexPolygon2d convexPolygon)
   {
      Point3f[] points = new Point3f[convexPolygon.getNumberOfVertices()];
      int reverseIndex = convexPolygon.getNumberOfVertices();
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = convexPolygon.getVertex(--reverseIndex);
         points[i] = new Point3f((float) vertex.getX(), (float) vertex.getY(), 0.0f);
      }

      return Polygon(points);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b> 
    * @param polygonPoints the vertices of the polygon.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder Polygon(Point3d[] polygonPoints)
   {
      Point3f[] points = makePoint3fArrayFromPoint3dArray(polygonPoints);

      return Polygon(points);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <b> It is assumed that the polygon is convex and counter-clockwise ordered. </b> 
    * @param polygonPoints the vertices of the polygon.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder Polygon(Point3f[] polygonPoints)
   {
      // Assume convexity and ccw.
      int[] polygonIndices = new int[polygonPoints.length];
      int[] polygonStripCounts = new int[] {polygonPoints.length};

      for (int i = 0; i < polygonPoints.length; i++)
      {
         polygonIndices[i] = i;
      }

      return new MeshDataHolder(polygonPoints, generateInterpolatedTexturePoints(polygonPoints.length), polygonIndices, polygonStripCounts);
   }

   /**
    * Create a triangle mesh for the given polygon 2d and extrude it along the z-axis.
    * @param convexPolygon2d the polygon to create a mesh from.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder ExtrudedPolygon(ConvexPolygon2d convexPolygon2d, double extrusionHeight)
   {
      Point2d[] points = new Point2d[convexPolygon2d.getNumberOfVertices()];
      int reverseIndex = convexPolygon2d.getNumberOfVertices();
      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         points[i] = new Point2d(convexPolygon2d.getVertex(--reverseIndex));
      }

      return ExtrudedPolygon(points, extrusionHeight);
   }

   /**
    * Create a triangle mesh for the given polygon 2d and extrude it along the z-axis.
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b>
    * @param polygonPoints the vertices of the polygon.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder ExtrudedPolygon(List<Point2d> polygonPoints, double extrusionHeight)
   {
      Point2d[] points = new Point2d[polygonPoints.size()];
      int i = 0;
      for (Point2d point2d : polygonPoints)
      {
         points[i++] = new Point2d(point2d);
      }

      return ExtrudedPolygon(points, extrusionHeight);
   }

   /**
    * Create a triangle mesh for the given polygon 2d and extrude it along the z-axis.
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b>
    * <p> 
    * TODO: Figure out how to texture an extruded polygon!
    * @param polygonPoints the vertices of the polygon.
    * @return the created triangle mesh.
    */
   public static MeshDataHolder ExtrudedPolygon(Point2d[] clockwiseOrderedConvexPolygonPoints, double extrusionHeight)
   {
      int N = clockwiseOrderedConvexPolygonPoints.length;

      Point3f vertices[] = new Point3f[4 * N + 2];
      Vector3f normals[] = new Vector3f[4 * N + 2];
      TexCoord2f[] texturePoints = new TexCoord2f[4 * N + 2];

      Point2d average = new Point2d();
      for (Point2d polygonPoint : clockwiseOrderedConvexPolygonPoints)
         average.add(polygonPoint);
      average.scale(1.0 / N);

      for (int i = 0; i < N; i++)
      {
         float vertexX = (float) (clockwiseOrderedConvexPolygonPoints[i].getX());
         float vertexY = (float) (clockwiseOrderedConvexPolygonPoints[i].getY());
         float normalX = vertexX - (float) average.getX();
         float normalY = vertexY - (float) average.getY();
         float normalLength = (float) Math.sqrt(normalX * normalX + normalY * normalY);
         normalX /= normalLength;
         normalY /= normalLength;

         // Vertices for bottom face
         vertices[i] = new Point3f(vertexX, vertexY, 0.0f);
         normals[i] = new Vector3f(0.0f, 0.0f, -1.0f);
         texturePoints[i] = new TexCoord2f();

         // Vertices for top face
         vertices[i + N] = new Point3f(vertexX, vertexY, (float) extrusionHeight);
         normals[i + N] = new Vector3f(0.0f, 0.0f, 1.0f);
         texturePoints[i + N] = new TexCoord2f();

         // Vertices for side faces
         // Bottom
         vertices[i + 2 * N] = new Point3f(vertexX, vertexY, 0.0f);
         normals[i + 2 * N] = new Vector3f(normalX, normalY, 0.0f);
         texturePoints[i + 2 * N] = new TexCoord2f();

         // Top
         vertices[i + 3 * N] = new Point3f(vertexX, vertexY, (float) extrusionHeight);
         normals[i + 3 * N] = new Vector3f(normalX, normalY, 0.0f);
         texturePoints[i + 3 * N] = new TexCoord2f();
      }

      // Bottom center
      int bottomCenterIndex = 4 * N;
      vertices[bottomCenterIndex] = new Point3f((float) average.getX(), (float) average.getY(), 0.0f);
      normals[bottomCenterIndex] = new Vector3f(0.0f, 0.0f, -1.0f);
      texturePoints[bottomCenterIndex] = new TexCoord2f();

      // Top center
      int topCenterIndex = 4 * N + 1;
      vertices[topCenterIndex] = new Point3f((float) average.getX(), (float) average.getY(), (float) extrusionHeight);
      normals[topCenterIndex] = new Vector3f(0.0f, 0.0f, 1.0f);
      texturePoints[topCenterIndex] = new TexCoord2f();

      int numberOfTriangles = 4 * N;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      for (int i = 0; i < N; i++)
      {
         // Bottom face
         triangleIndices[index++] = (i + 1) % N;
         triangleIndices[index++] = i;
         triangleIndices[index++] = bottomCenterIndex;

         // Top face
         triangleIndices[index++] = topCenterIndex;
         triangleIndices[index++] = i + N;
         triangleIndices[index++] = (i + 1) % N + N;

         // Side face
         // Lower triangle
         triangleIndices[index++] = i + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = i + 3 * N;
         // Upper triangle
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 3 * N;
         triangleIndices[index++] = i + 3 * N;
      }

      int[] polygonStripCounts = new int[numberOfTriangles];
      Arrays.fill(polygonStripCounts, 3);

      return new MeshDataHolder(vertices, texturePoints, triangleIndices, polygonStripCounts, normals);
   }

   public static MeshDataHolder HemiEllipsoid(double xRadius, double yRadius, double zRadius, int latitudeN, int longitudeN)
   {
      return HemiEllipsoid((float) xRadius, (float) yRadius, (float) zRadius, latitudeN, longitudeN);
   }

   public static MeshDataHolder HemiEllipsoid(float xRadius, float yRadius, float zRadius, int latitudeN, int longitudeN)
   {
      // Reminder of longitude and latitude: http://www.geographyalltheway.com/ks3_geography/maps_atlases/longitude_latitude.htm
      Point3f points[] = new Point3f[(latitudeN + 1) * longitudeN + 2];
      Vector3f[] normals = new Vector3f[(latitudeN + 1) * longitudeN + 2];
      TexCoord2f textPoints[] = new TexCoord2f[(latitudeN + 1) * longitudeN + 2];

      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         float longitudeAngle = (float) (2.0 * Math.PI * ((float) longitudeIndex / (float) longitudeN));

         for (int latitudeIndex = 0; latitudeIndex < latitudeN; latitudeIndex++)
         {
            float latitudeAngle = (float) (Math.PI / 2.0 * ((float) latitudeIndex / (float) latitudeN));

            int currentIndex = latitudeIndex * longitudeN + longitudeIndex;
            float vertexX = (float) (xRadius * Math.cos(longitudeAngle) * Math.cos(latitudeAngle));
            float vertexY = (float) (yRadius * Math.sin(longitudeAngle) * Math.cos(latitudeAngle));
            float vertexZ = (float) (zRadius * Math.sin(latitudeAngle));
            points[currentIndex] = new Point3f(vertexX, vertexY, vertexZ);

            float normalX = (float) (Math.cos(longitudeAngle) * Math.cos(latitudeAngle));
            float normalY = (float) (Math.sin(longitudeAngle) * Math.cos(latitudeAngle));
            float normalZ = (float) (Math.sin(latitudeAngle));
            normals[currentIndex] = new Vector3f(normalX, normalY, normalZ);

            float textureX = (float) (longitudeAngle / (2.0 * Math.PI));
            float textureY = (float) (0.5 * Math.sin(latitudeAngle) + 0.5);
            textPoints[currentIndex] = new TexCoord2f(textureX, textureY);
         }
      }

      // Bottom side
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         float longitudeAngle = (float) (2.0 * Math.PI * ((float) longitudeIndex / (float) longitudeN));

         int currentIndex = latitudeN * longitudeN + longitudeIndex;
         float vertexX = (float) (xRadius * Math.cos(longitudeAngle));
         float vertexY = (float) (yRadius * Math.sin(longitudeAngle));
         float vertexZ = 0.0f;
         points[currentIndex] = new Point3f(vertexX, vertexY, vertexZ);

         normals[currentIndex] = new Vector3f(0.0f, 0.0f, -1.0f);

         float textureX = (float) (longitudeAngle / (2.0 * Math.PI));
         textPoints[currentIndex] = new TexCoord2f(textureX, 0.5f);
      }

      // North pole
      int northPoleIndex = (latitudeN + 1) * longitudeN;
      points[northPoleIndex] = new Point3f(0.0f, 0.0f, zRadius);
      normals[northPoleIndex] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[northPoleIndex] = new TexCoord2f(1.0f, 1.0f);

      // Bottom center
      int bottomCenterIndex = (latitudeN + 1) * longitudeN + 1;
      points[bottomCenterIndex] = new Point3f(0.0f, 0.0f, 0.0f);
      normals[bottomCenterIndex] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[bottomCenterIndex] = new TexCoord2f(0.5f, 0.5f);

      int numberOfTriangles = 2 * latitudeN * longitudeN + 2 * longitudeN;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Mid-latitude faces
      for (int latitudeIndex = 0; latitudeIndex < latitudeN - 1; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
            int nextLatitudeIndex = (latitudeIndex + 1);

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
         }
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = northPoleIndex;
         triangleIndices[index++] = (latitudeN - 1) * longitudeN + longitudeIndex;
         triangleIndices[index++] = (latitudeN - 1) * longitudeN + nextLongitudeIndex;
      }

      // Bottom face
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = latitudeN * longitudeN + nextLongitudeIndex;
         triangleIndices[index++] = latitudeN * longitudeN + longitudeIndex;
         triangleIndices[index++] = bottomCenterIndex;
      }

      int[] pStripCounts = new int[numberOfTriangles];
      Arrays.fill(pStripCounts, 3);

      return new MeshDataHolder(points, textPoints, triangleIndices, pStripCounts, normals);
   }

   public static MeshDataHolder Cylinder(double radius, double height, int N)
   {
      return Cylinder((float) radius, (float) height, N);
   }

   public static MeshDataHolder Cylinder(float radius, float height, int N)
   {
      Point3f points[] = new Point3f[4 * N + 2];
      Vector3f normals[] = new Vector3f[4 * N + 2];
      TexCoord2f textPoints[] = new TexCoord2f[4 * N + 2];

      for (int i = 0; i < N; i++)
      {
         double angle = i * 2.0 * Math.PI / N;
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float vertexX = (radius * cosAngle);
         float vertexY = (radius * sinAngle);

         // Bottom vertices
         points[i] = new Point3f(vertexX, vertexY, 0.0f);
         normals[i] = new Vector3f(0.0f, 0.0f, -1.0f);
         textPoints[i] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top vertices
         points[i + N] = new Point3f(vertexX, vertexY, height);
         normals[i + N] = new Vector3f(0.0f, 0.0f, 1.0f);
         textPoints[i + N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Outer vertices
         // Bottom
         points[i + 2 * N] = new Point3f(vertexX, vertexY, 0.0f);
         normals[i + 2 * N] = new Vector3f(cosAngle, sinAngle, 0.0f);
         textPoints[i + 2 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top
         points[i + 3 * N] = new Point3f(vertexX, vertexY, height);
         normals[i + 3 * N] = new Vector3f(cosAngle, sinAngle, 0.0f);
         textPoints[i + 3 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
      }

      // Center of bottom cap
      points[4 * N] = new Point3f(0.0f, 0.0f, 0.0f);
      normals[4 * N] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[4 * N] = new TexCoord2f(0.5f, 0.5f);
      // Center of top cap
      points[4 * N + 1] = new Point3f(0.0f, 0.0f, height);
      normals[4 * N + 1] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[4 * N + 1] = new TexCoord2f(0.5f, 0.5f);

      int numberOfTriangles = 4 * N;
      int[] triangleIndices = new int[6 * numberOfTriangles];

      int index = 0;

      for (int i = 0; i < N; i++)
      { // The bottom cap
         triangleIndices[index++] = (i + 1) % N;
         triangleIndices[index++] = i;
         triangleIndices[index++] = 4 * N;
      }

      for (int i = 0; i < N; i++)
      { // The top cap
         triangleIndices[index++] = 4 * N + 1;
         triangleIndices[index++] = i + N;
         triangleIndices[index++] = (i + 1) % N + N;
      }

      for (int i = 0; i < N; i++)
      { // The cylinder part
           // Lower triangle
         triangleIndices[index++] = i + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = i + 3 * N;
         // Upper triangle
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 3 * N;
         triangleIndices[index++] = i + 3 * N;
      }

      int[] pStripCounts = new int[numberOfTriangles];
      Arrays.fill(pStripCounts, 3);

      return new MeshDataHolder(points, textPoints, triangleIndices, pStripCounts, normals);
   }

   public static MeshDataHolder Cone(double height, double radius, int N)
   {
      return Cone((float) height, (float) radius, N);
   }

   public static MeshDataHolder Cone(float height, float radius, int N)
   {
      Point3f[] vertices = new Point3f[2 * N + 2];
      Vector3f[] normals = new Vector3f[2 * N + 2];
      TexCoord2f[] textureCoordinates = new TexCoord2f[2 * N + 2];

      // This is equal to half of the opening angle of the cone at its top. Used to compute the normals.
      float slopeAngle = (float) Math.atan2(radius, height);
      float cosSlopeAngle = (float) Math.cos(slopeAngle);
      float sinSlopeAngle = (float) Math.sin(slopeAngle);

      for (int i = 0; i < N; i++)
      {
         double angle = i * 2.0 * Math.PI / N;
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float vertexX = radius * cosAngle;
         float vertexY = radius * sinAngle;

         // Vertices for the bottom part.
         vertices[i] = new Point3f(vertexX, vertexY, 0.0f);
         normals[i] = new Vector3f(0.0f, 0.0f, -1.0f);
         textureCoordinates[i] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
         // Vertices for the side part.
         vertices[i + N] = new Point3f(vertexX, vertexY, 0.0f);
         normals[i + N] = new Vector3f(cosSlopeAngle * cosAngle, cosSlopeAngle * sinAngle, sinSlopeAngle);
         textureCoordinates[i + N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
      }

      // The top peak
      int peakIndex = 2 * N;
      vertices[peakIndex] = new Point3f(0.0f, 0.0f, height);
      normals[peakIndex] = new Vector3f(0.0f, 0.0f, 1.0f);
      textureCoordinates[peakIndex] = new TexCoord2f(0.5f, 0.5f);

      // The center of the bottom
      int bottomCenterIndex = 2 * N + 1;
      vertices[bottomCenterIndex] = new Point3f(0.0f, 0.0f, 0.0f);
      normals[bottomCenterIndex] = new Vector3f(0.0f, 0.0f, -1.0f);
      textureCoordinates[bottomCenterIndex] = new TexCoord2f(0.5f, 0.5f);

      int numberOfTriangles = 2 * N;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      for (int i = 0; i < N; i++)
      {
         // The bottom face
         triangleIndices[index++] = bottomCenterIndex;
         triangleIndices[index++] = (i + 1) % N;
         triangleIndices[index++] = i;

         // The side faces
         triangleIndices[index++] = i + N;
         triangleIndices[index++] = (i + 1) % N + N;
         triangleIndices[index++] = peakIndex;
      }

      int[] pStripCounts = new int[numberOfTriangles];
      Arrays.fill(pStripCounts, 3);

      return new MeshDataHolder(vertices, textureCoordinates, triangleIndices, pStripCounts, normals);
   }

   public static MeshDataHolder GenTruncatedCone(double height, double bx, double by, double tx, double ty, int N)
   {
      return GenTruncatedCone((float) height, (float) bx, (float) by, (float) tx, (float) ty, N);
   }

   public static MeshDataHolder GenTruncatedCone(float height, float bx, float by, float tx, float ty, int N)
   {
      Point3f points[] = new Point3f[2 * N];
      TexCoord2f[] textPoints = new TexCoord2f[2 * N];

      for (int i = 0; i < N; i++)
      {
         points[i] = new Point3f((float) (bx * Math.cos(i * 2.0 * Math.PI / N)), (float) (by * Math.sin(i * 2.0 * Math.PI / N)), 0.0f);
         textPoints[i] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N) + 0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
      }

      for (int i = 0; i < N; i++)
      {
         points[i + N] = new Point3f((float) (tx * Math.cos(i * 2.0 * Math.PI / N)), (float) (ty * Math.sin(i * 2.0 * Math.PI / N)), height);
         textPoints[i + N] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N) + 0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
      }

      int[] polygonIndices = new int[N + N + 4 * N];

      int index = 0;

      // Bottom
      for (int i = 0; i < N; i++)
      {
         polygonIndices[index] = N - 1 - i;
         index = index + 1;
      }

      // Top
      for (int i = 0; i < N; i++)
      {
         polygonIndices[index] = N + i;
         index = index + 1;
      }

      for (int i = 0; i < N - 1; i++)
      {
         polygonIndices[index] = i;
         polygonIndices[index + 1] = i + 1;
         polygonIndices[index + 2] = N + i + 1;
         polygonIndices[index + 3] = N + i;
         index = index + 4;
      }

      polygonIndices[index] = N - 1;
      polygonIndices[index + 1] = 0;
      polygonIndices[index + 2] = N;
      polygonIndices[index + 3] = 2 * N - 1;

      int[] pStripCounts = new int[2 + N];

      pStripCounts[0] = N;
      pStripCounts[1] = N;

      for (int i = 2; i < N + 2; i++)
      {
         pStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   public static MeshDataHolder ArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, int N)
   {
      return ArcTorus((float) startAngle, (float) endAngle, (float) majorRadius, (float) minorRadius, N);
   }

   public static MeshDataHolder ArcTorus(float startAngle, float endAngle, float majorRadius, float minorRadius, int N)
   {
      Point3f points[] = new Point3f[N * N];
      TexCoord2f[] textPoints = new TexCoord2f[N * N];
      double angle1, angle2;
      double cenX, cenY;
      double pX, pY, pZ;
      float texY, texX;

      for (int i = 0; i < N; i++)
      {
         angle1 = startAngle + i * (endAngle - startAngle) / (N - 1);
         cenX = majorRadius * Math.cos(angle1);
         cenY = majorRadius * Math.sin(angle1);

         texY = (float) i / (float) N;

         for (int j = 0; j < N; j++)
         {
            angle2 = j * 2.0 * Math.PI / N;
            pX = cenX + minorRadius * Math.cos(angle1) * Math.cos(angle2);
            pY = cenY + minorRadius * Math.sin(angle1) * Math.cos(angle2);
            pZ = minorRadius * Math.sin(angle2);
            points[i * N + j] = new Point3f((float) pX, (float) pY, (float) pZ);

            texX = (float) j / (float) N;
            textPoints[i * N + j] = new TexCoord2f(texX, texY);
         }
      }

      int[] polygonIndices = new int[4 * (N - 1) * N];

      int index = 0;

      // Bottom
      for (int i = 0; i < N - 1; i++)
      {
         for (int j = 0; j < N - 1; j++)
         {
            polygonIndices[index + 3] = (i * N) + j;
            polygonIndices[index + 2] = (i * N) + j + 1;
            polygonIndices[index + 1] = (i + 1) * N + j + 1;
            polygonIndices[index] = (i + 1) * N + j;

            index = index + 4;
         }

         polygonIndices[index + 3] = (i * N) + N - 1;
         polygonIndices[index + 2] = (i * N);
         polygonIndices[index + 1] = (i + 1) * N;
         polygonIndices[index] = (i + 1) * N + N - 1;
         index = index + 4;
      }

      int[] pStripCounts = new int[(N - 1) * N];

      for (int i = 0; i < (N - 1) * N; i++)
      {
         pStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   public static MeshDataHolder Cube(double lx, double ly, double lz, boolean centered)
   {
      return (Cube((float) lx, (float) ly, (float) lz, centered));
   }

   public static MeshDataHolder Cube(float lx, float ly, float lz, boolean centered)
   {
      Point3f points[] = new Point3f[24];
      Vector3f[] normals = new Vector3f[24];
      TexCoord2f textPoints[] = new TexCoord2f[24];

      float za = centered ? -lz / 2f : 0;
      float zb = centered ? lz / 2f : lz;

      // Bottom vertices for bottom face
      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, za);
      normals[0] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, za);
      normals[1] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, za);
      normals[2] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, za);
      normals[3] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      // Top vertices for top face
      points[4] = new Point3f(-lx / 2.0f, -ly / 2.0f, zb);
      normals[4] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[4] = new TexCoord2f(0.0f, 0.0f);
      points[5] = new Point3f(lx / 2.0f, -ly / 2.0f, zb);
      normals[5] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[5] = new TexCoord2f(1.0f, 0.0f);
      points[6] = new Point3f(lx / 2.0f, ly / 2.0f, zb);
      normals[6] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[6] = new TexCoord2f(1.0f, 1.0f);
      points[7] = new Point3f(-lx / 2.0f, ly / 2.0f, zb);
      normals[7] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[7] = new TexCoord2f(0.0f, 1.0f);

      // Left face vertices
      points[8] = points[2];
      normals[8] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[8] = textPoints[2];
      points[9] = points[3];
      normals[9] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[9] = textPoints[3];
      points[10] = points[6];
      normals[10] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[10] = textPoints[6];
      points[11] = points[7];
      normals[11] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[11] = textPoints[7];

      // Right face vertices
      points[12] = points[0];
      normals[12] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[12] = textPoints[0];
      points[13] = points[1];
      normals[13] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[13] = textPoints[1];
      points[14] = points[4];
      normals[14] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[14] = textPoints[4];
      points[15] = points[5];
      normals[15] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[15] = textPoints[5];

      // Front face vertices
      points[16] = points[0];
      normals[16] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[16] = textPoints[0];
      points[17] = points[3];
      normals[17] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[17] = textPoints[3];
      points[18] = points[4];
      normals[18] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[18] = textPoints[4];
      points[19] = points[7];
      normals[19] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[19] = textPoints[7];

      // Back face vertices
      points[20] = points[1];
      normals[20] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[20] = textPoints[1];
      points[21] = points[2];
      normals[21] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[21] = textPoints[2];
      points[22] = points[5];
      normals[22] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[22] = textPoints[5];
      points[23] = points[6];
      normals[23] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[23] = textPoints[6];

      int numberOfTriangles = 2 * 6;

      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Bottom face (face vertices 0, 1, 2, 3)
      triangleIndices[index++] = 2;
      triangleIndices[index++] = 1;
      triangleIndices[index++] = 0;

      triangleIndices[index++] = 3;
      triangleIndices[index++] = 2;
      triangleIndices[index++] = 0;

      // Top face (face vertices 4, 5, 6, 7)
      triangleIndices[index++] = 4;
      triangleIndices[index++] = 5;
      triangleIndices[index++] = 6;

      triangleIndices[index++] = 4;
      triangleIndices[index++] = 6;
      triangleIndices[index++] = 7;

      // Left face (face vertices 8, 9, 10, 11)
      triangleIndices[index++] = 8;
      triangleIndices[index++] = 11;
      triangleIndices[index++] = 10;
      
      triangleIndices[index++] = 8;
      triangleIndices[index++] = 9;
      triangleIndices[index++] = 11;

      // Right face (face vertices 12, 13, 14, 15)
      triangleIndices[index++] = 15;
      triangleIndices[index++] = 14;
      triangleIndices[index++] = 13;
      
      triangleIndices[index++] = 14;
      triangleIndices[index++] = 12;
      triangleIndices[index++] = 13;

      // Front face (face vertices 16, 17, 18, 19)
      triangleIndices[index++] = 16;
      triangleIndices[index++] = 19;
      triangleIndices[index++] = 17;
      
      triangleIndices[index++] = 16;
      triangleIndices[index++] = 18;
      triangleIndices[index++] = 19;

      // Back face (face vertices 20, 21, 22, 23)
      triangleIndices[index++] = 20;
      triangleIndices[index++] = 23;
      triangleIndices[index++] = 22;
      
      triangleIndices[index++] = 20;
      triangleIndices[index++] = 21;
      triangleIndices[index++] = 23;
      

      int[] polygonStripCounts = new int[numberOfTriangles];
      Arrays.fill(polygonStripCounts, 3);

      return new MeshDataHolder(points, textPoints, triangleIndices, polygonStripCounts, normals);
   }

   public static MeshDataHolder FlatRectangle(double xMin, double yMin, double xMax, double yMax, double z)
   {
      return FlatRectangle((float) xMin, (float) yMin, (float) xMax, (float) yMax, (float) z);
   }

   public static MeshDataHolder FlatRectangle(float xMin, float yMin, float xMax, float yMax, float z)
   {
      Point3f[] points = new Point3f[4];
      Vector3f[] normals = new Vector3f[4];
      TexCoord2f[] textPoints = new TexCoord2f[4];

      points[0] = new Point3f(xMin, yMin, z);
      points[1] = new Point3f(xMax, yMin, z);
      points[2] = new Point3f(xMax, yMax, z);
      points[3] = new Point3f(xMin, yMax, z);

      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      normals[0] = new Vector3f(0.0f, 0.0f, 1.0f);
      normals[1] = new Vector3f(0.0f, 0.0f, 1.0f);
      normals[2] = new Vector3f(0.0f, 0.0f, 1.0f);
      normals[3] = new Vector3f(0.0f, 0.0f, 1.0f);

      int[] triangleIndices = new int[3 * 2];
      int index = 0;
      triangleIndices[index++] = 0;
      triangleIndices[index++] = 3;
      triangleIndices[index++] = 1;

      triangleIndices[index++] = 3;
      triangleIndices[index++] = 2;
      triangleIndices[index++] = 1;

      int[] polygonStripCounts = {3, 3};

      return new MeshDataHolder(points, textPoints, triangleIndices, polygonStripCounts, normals);
   }

   public static MeshDataHolder Rectangle(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, double x3,
         double y3, double z3)
   {
      return Rectangle((float) x0, (float) y0, (float) z0, (float) x1, (float) y1, (float) z1, (float) x2, (float) y2, (float) z2, (float) x3, (float) y3,
            (float) z3);
   }

   public static MeshDataHolder Rectangle(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3,
         float z3)
   {
      Point3f points[] = new Point3f[4];

      TexCoord2f textPoints[] = new TexCoord2f[4];

      points[0] = new Point3f(x0, y0, z0);
      points[1] = new Point3f(x1, y1, z1);
      points[2] = new Point3f(x2, y2, z2);
      points[3] = new Point3f(x3, y3, z3);

      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      int[] polygonIndices = new int[4];
      for (int i = 0; i < polygonIndices.length; i++)
      {
         polygonIndices[i] = i;
      }

      int[] polygonStripCounts = {4};

      return new MeshDataHolder(points, textPoints, polygonIndices, polygonStripCounts);
   }

   public static MeshDataHolder Wedge(double lx, double ly, double lz)
   {
      return (Wedge((float) lx, (float) ly, (float) lz));
   }

   public static MeshDataHolder Wedge(float lx, float ly, float lz)
   {
      Point3f points[] = new Point3f[6];

      TexCoord2f textPoints[] = new TexCoord2f[6];

      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);

      points[4] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      points[5] = new Point3f(lx / 2.0f, ly / 2.0f, lz);

      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      textPoints[4] = new TexCoord2f(0.0f, 1.0f);
      textPoints[5] = new TexCoord2f(1.0f, 1.0f);

      int[] polygonIndices = new int[3 * 4 + 2 * 3];

      int index = 0;

      polygonIndices[index] = 3;
      polygonIndices[index + 1] = 2;
      polygonIndices[index + 2] = 1;
      polygonIndices[index + 3] = 0;

      index = index + 4;

      polygonIndices[index] = 2;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 4;
      polygonIndices[index + 3] = 1;

      index = index + 4;

      polygonIndices[index] = 4;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 3;
      polygonIndices[index + 3] = 0;

      index = index + 4;

      polygonIndices[index] = 1;
      polygonIndices[index + 1] = 4;
      polygonIndices[index + 2] = 0;

      index = index + 3;

      polygonIndices[index] = 3;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 2;

      index = index + 3;

      int[] pStripCounts = new int[5];

      for (int i = 0; i < 3; i++)
      {
         pStripCounts[i] = 4;
      }

      for (int i = 3; i < 5; i++)
      {
         pStripCounts[i] = 3;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   public static MeshDataHolder PyramidCube(double lx, double ly, double lz, double lh)
   {
      return (PyramidCube((float) lx, (float) ly, (float) lz, (float) lh));
   }

   public static MeshDataHolder PyramidCube(float lx, float ly, float lz, float lh)
   {
      Point3f points[] = new Point3f[10];

      TexCoord2f textPoints[] = new TexCoord2f[10];

      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      points[4] = new Point3f(-lx / 2.0f, -ly / 2.0f, lz);
      points[5] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      points[6] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      points[7] = new Point3f(-lx / 2.0f, ly / 2.0f, lz);

      points[8] = new Point3f(0.0f, 0.0f, lz + lh);
      points[9] = new Point3f(0.0f, 0.0f, -lh);

      textPoints[0] = new TexCoord2f(0.5f, 0.5f);
      textPoints[1] = new TexCoord2f(0.75f, 0.5f);
      textPoints[2] = new TexCoord2f(0.75f, 0.75f);
      textPoints[3] = new TexCoord2f(0.5f, 0.75f);
      textPoints[4] = new TexCoord2f(0.5f, 0.5f);
      textPoints[5] = new TexCoord2f(0.75f, 0.5f);
      textPoints[6] = new TexCoord2f(0.75f, 0.75f);
      textPoints[7] = new TexCoord2f(0.5f, 0.75f);

      textPoints[8] = new TexCoord2f(0.675f, 0.675f);
      textPoints[9] = new TexCoord2f(0.675f, 0.675f);

      int[] polygonIndices = new int[4 * 4 + 8 * 3];
      int index = 0;

      for (int i = 0; i <= 3; i++)
      {
         polygonIndices[index] = i;
         polygonIndices[index + 1] = (i + 1) % 4;
         polygonIndices[index + 2] = 4 + (i + 1) % 4;
         polygonIndices[index + 3] = 4 + i;

         index = index + 4;
      }

      for (int i = 0; i <= 3; i++)
      {
         polygonIndices[index] = (i + 1) % 4;
         polygonIndices[index + 1] = i;
         polygonIndices[index + 2] = 9;

         index = index + 3;
      }

      for (int i = 0; i <= 3; i++)
      {
         polygonIndices[index] = 4 + i;
         polygonIndices[index + 1] = 4 + (i + 1) % 4;
         polygonIndices[index + 2] = 8;

         index = index + 3;
      }

      int[] pStripCounts = new int[12];

      for (int i = 0; i < 4; i++)
      {
         pStripCounts[i] = 4;
      }

      for (int i = 4; i < 12; i++)
      {
         pStripCounts[i] = 3;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   //TODO: Figure out what a Gridded Polytope is and figure out how to draw them.
   //   public static MeshDataHolder griddedPolytope(Point3f[][] griddedPoints, double x_tiles, double y_tiles)
   //   {
   //      int firstSize = griddedPoints.length;
   //      int secondSize = griddedPoints[0].length;
   //
   //      int totalN = firstSize * 2 * secondSize;
   //
   //      Point3f[] coords = new Point3f[totalN];
   //
   //      
   //      int[] stripCounts = new int[firstSize];
   //      TexCoord2f[] textPoints = new TexCoord2f[totalN];
   //
   //      int index = 0;
   //      for (int i = 0; i < firstSize - 1; i++)
   //      {
   //         for (int j = 0; j < secondSize; j++)
   //         {
   //            coords[index] = new Point3f(griddedPoints[i + 1][j]);
   //            textPoints[index] = new TexCoord2f((float) (x_tiles * ((float) i + 1) / (firstSize)),
   //                                               (float) (y_tiles * (j) / (secondSize)));
   //            
   //            index++;
   //
   //            coords[index] = new Point3f(griddedPoints[i][j]);
   //            textPoints[index] = new TexCoord2f((float) (x_tiles * (i) / (firstSize)), (float) (y_tiles * (j) / (secondSize)));
   //            index++;
   //         }
   //      }
   //      
   //      for (int k = 0; k < secondSize; k++)
   //      {
   ////       stripCounts[k] = 2*(xPointsPerSide+1);
   //         stripCounts[k] = 2 * firstSize;
   //      }
   //
   //      int[] polygonIndices = new int[coords.length];
   //      for(int l = 0; l < firstSize; l++)
   //      {
   //         // TODO: Fill polygonIndices.
   //      }
   //      
   //      return new MeshDataHolder(coords, textPoints, polygonIndices, stripCounts);
   //   }
   //   

   private static TexCoord2f[] generateInterpolatedTexturePoints(int numPoints)
   {
      TexCoord2f[] textPoints = new TexCoord2f[numPoints];

      double distanceBetweenPoints = 4.0 / (numPoints);
      float[] xSides = {0.0f, 0.0f, 1.0f, 1.0f};
      float[] ySides = {0.0f, 1.0f, 1.0f, 0.0f};
      float positionAlongPerimeter;
      float positionAlongSide;
      int side;
      int secondPoint;
      float texCoordX, texCoordY;
      for (int i = 0; i < textPoints.length; i++)
      {
         positionAlongPerimeter = (float) distanceBetweenPoints * i;
         positionAlongSide = (float) (positionAlongPerimeter - Math.floor(positionAlongPerimeter));
         side = ((int) Math.floor(positionAlongPerimeter)) % 4;
         secondPoint = (side + 1) % 4;

         texCoordX = positionAlongSide * xSides[secondPoint] + (1.0f - positionAlongSide) * xSides[side];
         texCoordY = positionAlongSide * ySides[secondPoint] + (1.0f - positionAlongSide) * ySides[side];

         textPoints[i] = new TexCoord2f(texCoordX, texCoordY);
      }
      return textPoints;
   }

   private static Point3f[] makePoint3fArrayFromPoint3dArray(Point3d[] pPoints)
   {
      Point3f[] points3f = new Point3f[pPoints.length];
      int i = 0;
      for (Point3d point3d : pPoints)
      {
         points3f[i++] = new Point3f(point3d);
      }
      return points3f;
   }
}
