package us.ihmc.graphics3DAdapter.graphics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

import us.ihmc.robotics.MathTools;
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

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
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
      TexCoord2f[] textPoints = generateInterpolatedTexturePoints(polygonPoints.length);
      
      int numberOfTriangles = polygonPoints.length - 2;

      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;
      for (int j = 2; j < polygonPoints.length; j++)
      {
         triangleIndices[index++] = 0;
         triangleIndices[index++] = j - 1;
         triangleIndices[index++] = j;
      }

      Vector3f[] normals = findNormalsPerVertex(triangleIndices, polygonPoints);

      return new MeshDataHolder(polygonPoints, textPoints, triangleIndices, normals);
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

      return new MeshDataHolder(vertices, texturePoints, triangleIndices, normals);
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

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
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

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
   }

   public static MeshDataHolder Cone(double height, double radius, int N)
   {
      return Cone((float) height, (float) radius, N);
   }

   public static MeshDataHolder Cone(float height, float radius, int N)
   {
      Point3f[] vertices = new Point3f[3 * N + 1];
      Vector3f[] normals = new Vector3f[3 * N + 1];
      TexCoord2f[] textureCoordinates = new TexCoord2f[3 * N + 1];

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
         vertices[i + 2 * N] = new Point3f(0.0f, 0.0f, height);
         normals[i + 2 * N] = new Vector3f(cosSlopeAngle * cosAngle, cosSlopeAngle * sinAngle, sinSlopeAngle);
         textureCoordinates[i + 2 * N] = new TexCoord2f(0.5f, 0.5f);
      }

      // The center of the bottom
      int bottomCenterIndex = 3 * N;
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
         triangleIndices[index++] = i + 2 * N;
      }

      return new MeshDataHolder(vertices, textureCoordinates, triangleIndices, normals);
   }

   public static MeshDataHolder GenTruncatedCone(double height, double xBaseRadius, double yBaseRadius, double xTopRadius, double yTopRadius, int N)
   {
      return GenTruncatedCone((float) height, (float) xBaseRadius, (float) yBaseRadius, (float) xTopRadius, (float) yTopRadius, N);
   }

   public static MeshDataHolder GenTruncatedCone(float height, float xBaseRadius, float yBaseRadius, float xTopRadius, float yTopRadius, int N)
   {
      Point3f points[] = new Point3f[4 * N + 2];
      Vector3f[] normals = new Vector3f[4 * N + 2];
      TexCoord2f[] textPoints = new TexCoord2f[4 * N + 2];

      for (int i = 0; i < N; i++)
      {
         double angle = i * 2.0 * Math.PI / N;
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float baseX = xBaseRadius * cosAngle;
         float baseY = yBaseRadius * sinAngle;
         float topX = xTopRadius * cosAngle;
         float topY = yTopRadius * sinAngle;

         // Bottom face vertices
         points[i] = new Point3f(baseX, baseY, 0.0f);
         normals[i] = new Vector3f(0.0f, 0.0f, -1.0f);
         textPoints[i] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Top face vertices
         points[i + N] = new Point3f(topX, topY, height);
         normals[i + N] = new Vector3f(0.0f, 0.0f, 1.0f);
         textPoints[i + N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);

         // Cone face
         float currentBaseRadius = (float) Math.sqrt(baseX * baseX + baseY * baseY);
         float currentTopRadius = (float) Math.sqrt(topX * topX + topY * topY);
         float openingAngle = (float) Math.atan((currentBaseRadius - currentTopRadius) / height);
         float baseAngle = (float) Math.atan2(baseY, baseX);
         float topAngle = (float) Math.atan2(topY, topX);
         points[i + 2 * N] = new Point3f(baseX, baseY, 0.0f);
         normals[i + 2 * N] = new Vector3f((float) (Math.cos(baseAngle) * Math.cos(openingAngle)), (float) (Math.sin(baseAngle) * Math.cos(openingAngle)), (float) Math.sin(openingAngle));
         textPoints[i + 2 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
         points[i + 3 * N] = new Point3f(topX, topY, height);
         normals[i + 3 * N] = new Vector3f((float) (Math.cos(topAngle) * Math.cos(openingAngle)), (float) (Math.sin(topAngle) * Math.cos(openingAngle)), (float) Math.sin(openingAngle));
         textPoints[i + 3 * N] = new TexCoord2f(0.5f * cosAngle + 0.5f, 0.5f * sinAngle + 0.5f);
      }

      // Bottom center
      points[4 * N] = new Point3f(0.0f, 0.0f, 0.0f);
      normals[4 * N] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[4 * N] = new TexCoord2f(0.5f, 0.5f);
      // Top center
      points[4 * N + 1] = new Point3f(0.0f, 0.0f, height);
      normals[4 * N + 1] = new Vector3f(0.0f, 0.0f, 1.0f);
      textPoints[4 * N + 1] = new TexCoord2f(0.5f, 0.5f);

      int numberOfTriangles = 4 * N;
      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;

      for (int i = 0; i < N; i++)
      {
         // Bottom face
         triangleIndices[index++] = 4 * N;
         triangleIndices[index++] = (i + 1) % N;
         triangleIndices[index++] = i;
         // Top face
         triangleIndices[index++] = 4 * N + 1;
         triangleIndices[index++] = i + N;
         triangleIndices[index++] = (i + 1) % N + N;
         //Cone face: lower triangle
         triangleIndices[index++] = i + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = i + 3 * N;
         //Cone face: upper triangle
         triangleIndices[index++] = (i + 1) % N + 2 * N;
         triangleIndices[index++] = (i + 1) % N + 3 * N;
         triangleIndices[index++] = i + 3 * N;
      }

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
   }

   public static MeshDataHolder ArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, int N)
   {
      return ArcTorus((float) startAngle, (float) endAngle, (float) majorRadius, (float) minorRadius, N);
   }

   public static MeshDataHolder ArcTorus(float startAngle, float endAngle, float majorRadius, float minorRadius, int N)
   {
      float torusSpanAngle = endAngle - startAngle;
      boolean isClosed = MathTools.epsilonEquals(torusSpanAngle, 2.0 * Math.PI, 1.0e-3);

      // Make things a bit clearer.
      int majorN = N;
      // Make things a bit clearer.
      int minorN = N;

      float stepAngle = (endAngle - startAngle) / (isClosed ? majorN : majorN - 1);

      int numberOfVertices = isClosed ? majorN * minorN : majorN * minorN + 2 * (N + 1);
      Point3f points[] = new Point3f[numberOfVertices];
      Vector3f[] normals = new Vector3f[numberOfVertices];
      TexCoord2f[] textPoints = new TexCoord2f[numberOfVertices];

      float centerX, centerY;
      float pX, pY, pZ;
      float texY, texX;

      // Core part of the torus
      for (int majorIndex = 0; majorIndex < majorN; majorIndex++)
      {
         float majorAngle = startAngle + majorIndex * stepAngle;
         float cosMajorAngle = (float) Math.cos(majorAngle);
         float sinMajorAngle = (float) Math.sin(majorAngle);
         centerX = majorRadius * cosMajorAngle;
         centerY = majorRadius * sinMajorAngle;

         texY = (float) majorIndex / (float) majorN;

         for (int minorIndex = 0; minorIndex < minorN; minorIndex++)
         {
            int currentIndex = majorIndex * minorN + minorIndex;
            float minorAngle = minorIndex * 2.0f * (float) Math.PI / minorN;
            float cosMinorAngle = (float) Math.cos(minorAngle);
            float sinMinorAngle = (float) Math.sin(minorAngle);
            pX = centerX + minorRadius * cosMajorAngle * cosMinorAngle;
            pY = centerY + minorRadius * sinMajorAngle * cosMinorAngle;
            pZ = minorRadius * sinMinorAngle;
            points[currentIndex] = new Point3f(pX, pY, pZ);
            normals[currentIndex] = new Vector3f(cosMajorAngle * cosMinorAngle, sinMajorAngle * cosMinorAngle, sinMinorAngle);
            texX = (float) minorIndex / (float) minorN;
            textPoints[currentIndex] = new TexCoord2f(texX, texY);
         }
      }

      int lastMajorIndex = isClosed ? majorN : majorN - 1;
      int numberOfTriangles = 2 * lastMajorIndex * minorN;
      if (!isClosed)
         numberOfTriangles += 2 * minorN;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Torus core
      for (int majorIndex = 0; majorIndex < lastMajorIndex; majorIndex++)
      {
         for (int minorIndex = 0; minorIndex < minorN; minorIndex++)
         {
            int nextMajorIndex = (majorIndex + 1) % majorN;
            int nextMinorIndex = (minorIndex + 1) % minorN;

            triangleIndices[index++] = nextMajorIndex * minorN + minorIndex;
            triangleIndices[index++] = nextMajorIndex * minorN + nextMinorIndex;
            triangleIndices[index++] = majorIndex * minorN + nextMinorIndex;

            triangleIndices[index++] = nextMajorIndex * minorN + minorIndex;
            triangleIndices[index++] = majorIndex * minorN + nextMinorIndex;
            triangleIndices[index++] = majorIndex * minorN + minorIndex;
         }
      }

      // Close both ends when the torus is open
      if (!isClosed)
      {
         // First end
         float cosStartAngle = (float) Math.cos(startAngle);
         float sinStartAngle = (float) Math.sin(startAngle);
         centerX = majorRadius * cosStartAngle;
         centerY = majorRadius * sinStartAngle;

         for (int minorIndex = 0; minorIndex < minorN; minorIndex++)
         {
            int currentIndex = majorN * minorN + minorIndex;
            float minorAngle = minorIndex * 2.0f * (float) Math.PI / minorN;
            float cosMinorAngle = (float) Math.cos(minorAngle);
            float sinMinorAngle = (float) Math.sin(minorAngle);
            pX = centerX + minorRadius * cosStartAngle * cosMinorAngle;
            pY = centerY + minorRadius * sinStartAngle * cosMinorAngle;
            pZ = minorRadius * sinMinorAngle;
            points[currentIndex] = new Point3f(pX, pY, pZ);
            normals[currentIndex] = new Vector3f(sinStartAngle, -cosStartAngle, 0.0f);
            texX = (float) minorIndex / (float) minorN;
            textPoints[currentIndex] = new TexCoord2f(texX, 0.0f);
         }

         // First end center
         int firstEndCenterIndex = numberOfVertices - 2;
         points[firstEndCenterIndex] = new Point3f(centerX, centerY, 0.0f);
         normals[firstEndCenterIndex] = new Vector3f(sinStartAngle, -cosStartAngle, 0.0f);
         textPoints[firstEndCenterIndex] = new TexCoord2f(0.0f, 0.0f);

         // Second end
         float cosEndAngle = (float) Math.cos(endAngle);
         float sinEndAngle = (float) Math.sin(endAngle);
         centerX = majorRadius * cosEndAngle;
         centerY = majorRadius * sinEndAngle;

         for (int minorIndex = 0; minorIndex < minorN; minorIndex++)
         {
            int currentIndex = (majorN + 1) * minorN + minorIndex;
            float minorAngle = minorIndex * 2.0f * (float) Math.PI / minorN;
            float cosMinorAngle = (float) Math.cos(minorAngle);
            float sinMinorAngle = (float) Math.sin(minorAngle);
            pX = centerX + minorRadius * cosEndAngle * cosMinorAngle;
            pY = centerY + minorRadius * sinEndAngle * cosMinorAngle;
            pZ = minorRadius * sinMinorAngle;
            points[currentIndex] = new Point3f(pX, pY, pZ);
            normals[currentIndex] = new Vector3f(-sinEndAngle, cosEndAngle, 0.0f);
            texX = (float) minorIndex / (float) minorN;
            textPoints[currentIndex] = new TexCoord2f(texX, 1.0f);
         }

         // Second end center
         int secondEndCenterIndex = numberOfVertices - 1;
         points[secondEndCenterIndex] = new Point3f(centerX, centerY, 0.0f);
         normals[secondEndCenterIndex] = new Vector3f(-sinEndAngle, cosEndAngle, 0.0f);
         textPoints[secondEndCenterIndex] = new TexCoord2f(0.0f, 1.0f);

         // Setting up indices
         for (int minorIndex = 0; minorIndex < minorN; minorIndex++)
         {
            int nextMinorIndex = (minorIndex + 1) % minorN;

            triangleIndices[index++] = firstEndCenterIndex;
            triangleIndices[index++] = majorN * minorN + minorIndex;
            triangleIndices[index++] = majorN * minorN + nextMinorIndex;

            triangleIndices[index++] = secondEndCenterIndex;
            triangleIndices[index++] = (majorN + 1) * minorN + nextMinorIndex;
            triangleIndices[index++] = (majorN + 1) * minorN + minorIndex;
         }
      }

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
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

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
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

      return new MeshDataHolder(points, textPoints, triangleIndices, normals);
   }

   public static MeshDataHolder Wedge(double lx, double ly, double lz)
   {
      return (Wedge((float) lx, (float) ly, (float) lz));
   }

   public static MeshDataHolder Wedge(float lx, float ly, float lz)
   {
      Point3f[] points = new Point3f[18];
      Vector3f[] normals = new Vector3f[18];
      TexCoord2f[] textPoints = new TexCoord2f[18];

      // Bottom face vertices
      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[0] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[1] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      normals[2] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      normals[3] = new Vector3f(0.0f, 0.0f, -1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      // Back face vertices
      points[4] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      normals[4] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[4] = new TexCoord2f(0.0f, 1.0f);
      points[5] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      normals[5] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[5] = new TexCoord2f(1.0f, 1.0f);
      points[6] = new Point3f(points[2]);
      normals[6] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[6] = new TexCoord2f(textPoints[2]);
      points[7] = new Point3f(points[1]);
      normals[7] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[7] = new TexCoord2f(textPoints[1]);

      // Top face vertices
      float wedgeAngle = (float) Math.atan2(lz, lx);
      points[8] = new Point3f(points[0]);
      normals[8] = new Vector3f(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[8] = new TexCoord2f(textPoints[0]);
      points[9] = new Point3f(points[4]);
      normals[9] = new Vector3f(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[9] = new TexCoord2f(textPoints[4]);
      points[10] = new Point3f(points[5]);
      normals[10] = new Vector3f(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[10] = new TexCoord2f(textPoints[5]);
      points[11] = new Point3f(points[3]);
      normals[11] = new Vector3f(-(float) Math.sin(wedgeAngle), (float) Math.cos(wedgeAngle), 0.0f);
      textPoints[11] = new TexCoord2f(textPoints[3]);

      // Right face vertices
      points[12] = new Point3f(points[0]);
      normals[12] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[12] = new TexCoord2f(textPoints[0]);
      points[13] = new Point3f(points[1]);
      normals[13] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[13] = new TexCoord2f(textPoints[1]);
      points[14] = new Point3f(points[4]);
      normals[14] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[14] = new TexCoord2f(textPoints[4]);

      // Left face vertices
      points[15] = new Point3f(points[2]);
      normals[15] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[15] = new TexCoord2f(textPoints[2]);
      points[16] = new Point3f(points[3]);
      normals[16] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[16] = new TexCoord2f(textPoints[3]);
      points[17] = new Point3f(points[5]);
      normals[17] = new Vector3f(0.0f, 1.0f, 0.0f);
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

   public static MeshDataHolder PyramidCube(double lx, double ly, double lz, double lh)
   {
      return (PyramidCube((float) lx, (float) ly, (float) lz, (float) lh));
   }

   public static MeshDataHolder PyramidCube(float lx, float ly, float lz, float lh)
   {
      Point3f points[] = new Point3f[40];
      Vector3f[] normals = new Vector3f[40];
      TexCoord2f textPoints[] = new TexCoord2f[40];

      // Box front face
      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[0] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[0] = new TexCoord2f(0.5f, 0.5f);
      points[1] = new Point3f(-lx / 2.0f, -ly / 2.0f, lz);
      normals[1] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(0.5f, 0.5f);
      points[2] = new Point3f(-lx / 2.0f, ly / 2.0f, lz);
      normals[2] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[2] = new TexCoord2f(0.5f, 0.75f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      normals[3] = new Vector3f(-1.0f, 0.0f, 0.0f);
      textPoints[3] = new TexCoord2f(0.5f, 0.75f);

      // Box back face
      points[4] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[4] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[4] = new TexCoord2f(0.75f, 0.5f);
      points[5] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      normals[5] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[5] = new TexCoord2f(0.75f, 0.5f);
      points[6] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      normals[6] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[6] = new TexCoord2f(0.75f, 0.75f);
      points[7] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      normals[7] = new Vector3f(1.0f, 0.0f, 0.0f);
      textPoints[7] = new TexCoord2f(0.75f, 0.75f);

      // Box left face
      points[8] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      normals[8] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[8] = new TexCoord2f(0.5f, 0.75f);
      points[9] = new Point3f(-lx / 2.0f, ly / 2.0f, lz);
      normals[9] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[9] = new TexCoord2f(0.5f, 0.75f);
      points[10] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      normals[10] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[10] = new TexCoord2f(0.75f, 0.75f);
      points[11] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      normals[11] = new Vector3f(0.0f, 1.0f, 0.0f);
      textPoints[11] = new TexCoord2f(0.75f, 0.75f);

      // Box right face
      points[12] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[12] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[12] = new TexCoord2f(0.5f, 0.5f);
      points[13] = new Point3f(-lx / 2.0f, -ly / 2.0f, lz);
      normals[13] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[13] = new TexCoord2f(0.5f, 0.5f);
      points[14] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      normals[14] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[14] = new TexCoord2f(0.75f, 0.5f);
      points[15] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[15] = new Vector3f(0.0f, -1.0f, 0.0f);
      textPoints[15] = new TexCoord2f(0.75f, 0.5f);

      float frontBackAngle = (float) Math.atan2(lx / 2.0, lh);
      float leftRightAngle = (float) Math.atan2(ly / 2.0, lh);

      // Top pyramid
      // Front face
      points[16] = new Point3f(0.0f, 0.0f, lz + lh);
      normals[16] = new Vector3f(-(float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[16] = new TexCoord2f(0.675f, 0.675f);
      points[17] = new Point3f(-lx / 2.0f, -ly / 2.0f, lz);
      normals[17] = new Vector3f(-(float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[17] = new TexCoord2f(0.5f, 0.5f);
      points[18] = new Point3f(-lx / 2.0f, ly / 2.0f, lz);
      normals[18] = new Vector3f(-(float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[18] = new TexCoord2f(0.5f, 0.75f);

      // Back face
      points[19] = new Point3f(0.0f, 0.0f, lz + lh);
      normals[19] = new Vector3f((float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[19] = new TexCoord2f(0.675f, 0.675f);
      points[20] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      normals[20] = new Vector3f((float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[20] = new TexCoord2f(0.75f, 0.5f);
      points[21] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      normals[21] = new Vector3f((float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[21] = new TexCoord2f(0.75f, 0.75f);

      // Left face                                                                                       
      points[22] = new Point3f(0.0f, 0.0f, lz + lh);
      normals[22] = new Vector3f(0.0f, (float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[22] = new TexCoord2f(0.675f, 0.675f);
      points[23] = new Point3f(-lx / 2.0f, ly / 2.0f, lz);
      normals[23] = new Vector3f(0.0f, (float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[23] = new TexCoord2f(0.5f, 0.75f);
      points[24] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      normals[24] = new Vector3f(0.0f, (float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[24] = new TexCoord2f(0.75f, 0.75f);

      // Right face                                                                                      
      points[25] = new Point3f(0.0f, 0.0f, lz + lh);
      normals[25] = new Vector3f(0.0f, -(float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[25] = new TexCoord2f(0.675f, 0.675f);
      points[26] = new Point3f(-lx / 2.0f, -ly / 2.0f, lz);
      normals[26] = new Vector3f(0.0f, -(float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[26] = new TexCoord2f(0.5f, 0.5f);
      points[27] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      normals[27] = new Vector3f(0.0f, -(float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[27] = new TexCoord2f(0.75f, 0.5f);

      // Bottom pyramid
      // Front face
      points[28] = new Point3f(0.0f, 0.0f, -lh);
      normals[28] = new Vector3f(-(float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[28] = new TexCoord2f(0.675f, 0.675f);
      points[29] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[29] = new Vector3f(-(float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[29] = new TexCoord2f(0.5f, 0.5f);
      points[30] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      normals[30] = new Vector3f(-(float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[30] = new TexCoord2f(0.5f, 0.75f);

      // Back face
      points[31] = new Point3f(0.0f, 0.0f, -lh);
      normals[31] = new Vector3f((float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[31] = new TexCoord2f(0.675f, 0.675f);
      points[32] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[32] = new Vector3f((float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[32] = new TexCoord2f(0.75f, 0.5f);
      points[33] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      normals[33] = new Vector3f((float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[33] = new TexCoord2f(0.75f, 0.75f);

      // Left face                                                                                       
      points[34] = new Point3f(0.0f, 0.0f, -lh);
      normals[34] = new Vector3f(0.0f, (float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[34] = new TexCoord2f(0.675f, 0.675f);
      points[35] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      normals[35] = new Vector3f(0.0f, (float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[35] = new TexCoord2f(0.5f, 0.75f);
      points[36] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      normals[36] = new Vector3f(0.0f, (float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[36] = new TexCoord2f(0.75f, 0.75f);

      // Right face                                                                                       
      points[37] = new Point3f(0.0f, 0.0f, -lh);
      normals[37] = new Vector3f(0.0f, -(float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[37] = new TexCoord2f(0.675f, 0.675f);
      points[38] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[38] = new Vector3f(0.0f, -(float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[38] = new TexCoord2f(0.5f, 0.5f);
      points[39] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      normals[39] = new Vector3f(0.0f, -(float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[39] = new TexCoord2f(0.75f, 0.5f);

      int numberOfTriangles = 2 * 4 + 2 * 4;
      int[] polygonIndices = new int[3 * numberOfTriangles];
      int index = 0;

      // Box front face
      polygonIndices[index++] = 0;
      polygonIndices[index++] = 1;
      polygonIndices[index++] = 2;

      polygonIndices[index++] = 0;
      polygonIndices[index++] = 2;
      polygonIndices[index++] = 3;

      // Box back face
      polygonIndices[index++] = 4;
      polygonIndices[index++] = 6;
      polygonIndices[index++] = 5;

      polygonIndices[index++] = 4;
      polygonIndices[index++] = 7;
      polygonIndices[index++] = 6;

      // Box left face
      polygonIndices[index++] = 8;
      polygonIndices[index++] = 9;
      polygonIndices[index++] = 10;

      polygonIndices[index++] = 8;
      polygonIndices[index++] = 10;
      polygonIndices[index++] = 11;

      // Box right face
      polygonIndices[index++] = 12;
      polygonIndices[index++] = 14;
      polygonIndices[index++] = 13;

      polygonIndices[index++] = 12;
      polygonIndices[index++] = 15;
      polygonIndices[index++] = 14;

      // Top pyramid front face
      polygonIndices[index++] = 16;
      polygonIndices[index++] = 18;
      polygonIndices[index++] = 17;

      // Top pyramid back face
      polygonIndices[index++] = 19;
      polygonIndices[index++] = 20;
      polygonIndices[index++] = 21;

      // Top pyramid left face
      polygonIndices[index++] = 22;
      polygonIndices[index++] = 24;
      polygonIndices[index++] = 23;

      // Top pyramid right face
      polygonIndices[index++] = 25;
      polygonIndices[index++] = 26;
      polygonIndices[index++] = 27;

      // Bottom pyramid front face
      polygonIndices[index++] = 28;
      polygonIndices[index++] = 29;
      polygonIndices[index++] = 30;

      // Bottom pyramid back face
      polygonIndices[index++] = 31;
      polygonIndices[index++] = 33;
      polygonIndices[index++] = 32;

      // Bottom pyramid left face
      polygonIndices[index++] = 36;
      polygonIndices[index++] = 34;
      polygonIndices[index++] = 35;

      // Bottom pyramid right face
      polygonIndices[index++] = 37;
      polygonIndices[index++] = 39;
      polygonIndices[index++] = 38;

      return new MeshDataHolder(points, textPoints, polygonIndices, normals);
   }

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

   private static Vector3f[] findNormalsPerVertex(int[] indices, Point3f[] vertices)
   {
      Map<Integer, Set<Integer>> participatingFacesPerVertex = new LinkedHashMap<Integer, Set<Integer>>();

      for (int i = 0; i < indices.length; i++)
      {
         Set<Integer> vertexFacesSet = participatingFacesPerVertex.get(indices[i]);

         if (vertexFacesSet == null)
         {
            vertexFacesSet = new LinkedHashSet<Integer>();
            participatingFacesPerVertex.put(indices[i], vertexFacesSet);
         }

         // Abuse integer division assuming each face is a triangle => 3 indices per face;
         vertexFacesSet.add(i / 3);
      }

      Vector3f[] normalsPerFace = findNormalsPerFace(indices, vertices);

      int pos = 0;
      Vector3f[] normalsPerVertex = new Vector3f[vertices.length];

      for (int vertexIndex = 0; vertexIndex < vertices.length; vertexIndex++)
      {
         Set<Integer> participatingFaceIndices = participatingFacesPerVertex.get(vertexIndex);
         if (participatingFaceIndices == null)
            continue;

         Vector3f vertexNormal = new Vector3f();

         for (int face : participatingFaceIndices)
            vertexNormal.add(normalsPerFace[face]);

         float faces = (float) participatingFaceIndices.size();
         vertexNormal.scale(1.0f / faces);
         normalsPerVertex[pos++] = vertexNormal;
      }

      return normalsPerVertex;
   }

   private static Vector3f[] findNormalsPerFace(int[] indices, Point3f[] vertices)
   {
      Vector3f[] normalsPerFace = new Vector3f[indices.length / 3]; // Abuse integer division.

      Vector3f firstVector = new Vector3f();
      Vector3f secondVector = new Vector3f();
      Point3f[] faceVertices = new Point3f[3];

      for (int face = 0; face < normalsPerFace.length; face++)
      {
         normalsPerFace[face] = new Vector3f();

         for (int i = 0; i < faceVertices.length; i++)
         {
            faceVertices[i] = vertices[indices[face * 3 + i]];
         }

         firstVector.set(faceVertices[2]);
         firstVector.sub(faceVertices[0]);

         secondVector.set(faceVertices[2]);
         secondVector.sub(faceVertices[1]);

         normalsPerFace[face].cross(firstVector, secondVector);
         normalsPerFace[face].normalize();
      }

      return normalsPerFace;
   }

}
