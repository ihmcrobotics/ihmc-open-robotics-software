package us.ihmc.rdx.simulation.scs2;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.simulation.shapes.STPBox3D;

import java.util.*;
import java.util.function.DoubleFunction;

/**
 * This class provides factories to create generic meshes, i.e. {@code TriangleMesh3DDefinition}, to
 * represent a 3D shape.
 * <p>
 * The generic mesh can be used to construct a triangle mesh in the format of the graphics engine in
 * which it will be rendered.
 * </p>
 * <p>
 * The construction methods assumes the following coordinate system convention: x is pointing
 * forward, y pointing left, z pointing upward.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class TriangleMesh3DFactories
{
   private static final float TwoPi = 2.0f * (float) Math.PI;
   private static final float HalfPi = (float) Math.PI / 2.0f;

   private static final float SQRT3 = (float) Math.sqrt(3.0);
   private static final float SQRT6 = (float) Math.sqrt(6.0);
   private static final float HALF_SQRT3 = SQRT3 / 2.0f;
   private static final float THIRD_SQRT3 = SQRT3 / 3.0f;
   private static final float SIXTH_SQRT3 = SQRT3 / 6.0f;
   private static final float THIRD_SQRT6 = SQRT6 / 3.0f;
   private static final float FOURTH_SQRT6 = SQRT6 / 4.0f;

   private static final float ONE_THIRD = 1.0f / 3.0f;

   private TriangleMesh3DFactories()
   {
      // Prevent an object being generated.
   }

   /**
    * Tests the implementation of the given {@code description} and attempts to create the appropriate
    * triangle mesh.
    * 
    * @param description the geometry to create the mesh for.
    * @return the generic triangle mesh or {@code null} if the given {@code description} is not
    *         supported.
    */
   public static TriangleMesh3DDefinition TriangleMesh(GeometryDefinition description)
   {
      if (description == null)
         return null;

      TriangleMesh3DDefinition mesh = null;

      if (description instanceof TriangleMesh3DDefinition)
         return (TriangleMesh3DDefinition) description;
      else if (description instanceof ArcTorus3DDefinition)
         mesh = ArcTorus((ArcTorus3DDefinition) description);
      else if (description instanceof Box3DDefinition)
         mesh = Box((Box3DDefinition) description);
      else if (description instanceof Capsule3DDefinition)
         mesh = Capsule((Capsule3DDefinition) description);
      else if (description instanceof Cone3DDefinition)
         mesh = Cone((Cone3DDefinition) description);
      else if (description instanceof ConvexPolytope3DDefinition)
         mesh = ConvexPolytope((ConvexPolytope3DDefinition) description);
      else if (description instanceof Cylinder3DDefinition)
         mesh = Cylinder((Cylinder3DDefinition) description);
      else if (description instanceof Ellipsoid3DDefinition)
         mesh = Ellipsoid((Ellipsoid3DDefinition) description);
      else if (description instanceof ExtrudedPolygon2DDefinition)
         mesh = ExtrudedPolygon((ExtrudedPolygon2DDefinition) description);
      else if (description instanceof HemiEllipsoid3DDefinition)
         mesh = HemiEllipsoid((HemiEllipsoid3DDefinition) description);
      else if (description instanceof Polygon2DDefinition)
         mesh = Polygon((Polygon2DDefinition) description);
      else if (description instanceof Polygon3DDefinition)
         mesh = Polygon((Polygon2DDefinition) description);
      else if (description instanceof PyramidBox3DDefinition)
         mesh = PyramidBox((PyramidBox3DDefinition) description);
      else if (description instanceof Sphere3DDefinition)
         mesh = Sphere((Sphere3DDefinition) description);
      else if (description instanceof Tetrahedron3DDefinition)
         mesh = Tetrahedron((Tetrahedron3DDefinition) description);
      else if (description instanceof Torus3DDefinition)
         mesh = Torus((Torus3DDefinition) description);
      else if (description instanceof TruncatedCone3DDefinition)
         mesh = TruncatedCone((TruncatedCone3DDefinition) description);
      else if (description instanceof Ramp3DDefinition)
         mesh = Ramp((Ramp3DDefinition) description);

      if (mesh == null)
      {
         LogTools.error("Unrecognized " + GeometryDefinition.class.getSimpleName() + ": " + description.getClass().getSimpleName());
         return null;
      }
      else
      {
         if (description.getName() != null)
            mesh.setName(description.getName());
         return mesh;
      }
   }

   /**
    * Creates a triangle mesh for a 3D sphere.
    * <p>
    * The sphere is centered at the origin and is a UV sphere, see
    * <a href="https://en.wikipedia.org/wiki/UV_mapping">UV mapping</a>.
    * </p>
    *
    * @param description the description holding the sphere's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Sphere(Sphere3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Sphere(description.getRadius(), description.getResolution(), description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D sphere.
    * <p>
    * The sphere is centered at the origin and is a UV sphere, see
    * <a href="https://en.wikipedia.org/wiki/UV_mapping">UV mapping</a>.
    * </p>
    *
    * @param radius              the radius of the sphere. Each vertex is positioned at that distance
    *                            from the origin.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Sphere(double radius, int latitudeResolution, int longitudeResolution)
   {
      return Sphere((float) radius, latitudeResolution, longitudeResolution);
   }

   /**
    * Creates a triangle mesh for a 3D sphere.
    * <p>
    * The sphere is centered at the origin and is a UV sphere, see
    * <a href="https://en.wikipedia.org/wiki/UV_mapping">UV mapping</a>.
    * </p>
    *
    * @param radius              the radius of the sphere. Each vertex is positioned at that distance
    *                            from the origin.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Sphere(float radius, int latitudeResolution, int longitudeResolution)
   {
      return Ellipsoid(radius, radius, radius, latitudeResolution, longitudeResolution);
   }

   /**
    * Creates a triangle mesh for a 3D ellipsoid.
    * <p>
    * The ellipsoid is centered at the origin and the algorithm is similar to a UV sphere, see
    * <a href="https://en.wikipedia.org/wiki/UV_mapping">UV mapping</a>.
    * </p>
    *
    * @param description the description holding the ellipsoid's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Ellipsoid(Ellipsoid3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Ellipsoid(description.getRadiusX(),
                                                          description.getRadiusY(),
                                                          description.getRadiusZ(),
                                                          description.getResolution(),
                                                          description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D ellipsoid.
    * <p>
    * The ellipsoid is centered at the origin and the algorithm is similar to a UV sphere, see
    * <a href="https://en.wikipedia.org/wiki/UV_mapping">UV mapping</a>.
    * </p>
    *
    * @param radiusX             radius of the ellipsoid along the x-axis.
    * @param radiusY             radius of the ellipsoid along the y-axis.
    * @param radiusZ             radius of the ellipsoid along the z-axis.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Ellipsoid(double radiusX, double radiusY, double radiusZ, int latitudeResolution, int longitudeResolution)
   {
      return Ellipsoid((float) radiusX, (float) radiusY, (float) radiusZ, latitudeResolution, longitudeResolution);
   }

   /**
    * Creates a triangle mesh for a 3D ellipsoid.
    * <p>
    * The ellipsoid is centered at the origin and the algorithm is similar to a UV sphere, see
    * <a href="https://en.wikipedia.org/wiki/UV_mapping">UV mapping</a>.
    * </p>
    *
    * @param radiusX             radius of the ellipsoid along the x-axis.
    * @param radiusY             radius of the ellipsoid along the y-axis.
    * @param radiusZ             radius of the ellipsoid along the z-axis.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Ellipsoid(float radiusX, float radiusY, float radiusZ, int latitudeResolution, int longitudeResolution)
   {
      if (longitudeResolution % 2 == 1)
         longitudeResolution += 1;
      int nPointsLongitude = longitudeResolution + 1;
      int nPointsLatitude = latitudeResolution + 1;

      // Reminder of longitude and latitude: http://www.geographyalltheway.com/ks3_geography/maps_atlases/longitude_latitude.htm
      Point3D32 points[] = new Point3D32[nPointsLatitude * nPointsLongitude];
      Vector3D32[] normals = new Vector3D32[nPointsLatitude * nPointsLongitude];
      Point2D32 textPoints[] = new Point2D32[nPointsLatitude * nPointsLongitude];

      for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude; longitudeIndex++)
      {
         float longitudeAngle = TwoPi * ((float) longitudeIndex / (float) (nPointsLongitude - 1));
         float cosLongitude = (float) Math.cos(longitudeAngle);
         float sinLongitude = (float) Math.sin(longitudeAngle);
         float textureX = (float) longitudeIndex / (float) (nPointsLongitude - 1);

         for (int latitudeIndex = 1; latitudeIndex < nPointsLatitude - 1; latitudeIndex++)
         {
            float latitudeAngle = (float) (-HalfPi + Math.PI * ((float) latitudeIndex / (nPointsLatitude - 1.0f)));
            float cosLatitude = (float) Math.cos(latitudeAngle);
            float sinLatitude = (float) Math.sin(latitudeAngle);

            int currentIndex = latitudeIndex * nPointsLongitude + longitudeIndex;
            float normalX = cosLongitude * cosLatitude;
            float normalY = sinLongitude * cosLatitude;
            float normalZ = sinLatitude;
            float vertexX = radiusX * normalX;
            float vertexY = radiusY * normalY;
            float vertexZ = radiusZ * normalZ;
            points[currentIndex] = new Point3D32(vertexX, vertexY, vertexZ);
            normals[currentIndex] = new Vector3D32(normalX, normalY, normalZ);

            float textureY = 0.5f * (1.0f - sinLatitude);
            textPoints[currentIndex] = new Point2D32(textureX, textureY);
         }

         textureX += 0.5f / (nPointsLongitude - 1.0f);
         // South pole
         int southPoleIndex = longitudeIndex;
         points[southPoleIndex] = new Point3D32(0.0f, 0.0f, -radiusZ);
         normals[southPoleIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[southPoleIndex] = new Point2D32(textureX, 1.0f - 1.0f / 256.0f);

         // North pole
         int northPoleIndex = (nPointsLatitude - 1) * nPointsLongitude + longitudeIndex;
         points[northPoleIndex] = new Point3D32(0.0f, 0.0f, radiusZ);
         normals[northPoleIndex] = new Vector3D32(0.0f, 0.0f, 1.0f);
         textPoints[northPoleIndex] = new Point2D32(textureX, 1.0f / 256.0f);
      }

      int numberOfTriangles = 2 * ((nPointsLatitude - 2) * nPointsLongitude - 1);
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Mid-latitude faces
      for (int latitudeIndex = 1; latitudeIndex < nPointsLatitude - 2; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude - 1; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % nPointsLongitude;
            int nextLatitudeIndex = latitudeIndex + 1;

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * nPointsLongitude + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * nPointsLongitude + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * nPointsLongitude + longitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * nPointsLongitude + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * nPointsLongitude + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * nPointsLongitude + longitudeIndex;
         }
      }

      // South pole faces
      for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude - 1; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % nPointsLongitude;
         triangleIndices[index++] = longitudeIndex;
         triangleIndices[index++] = nPointsLongitude + nextLongitudeIndex;
         triangleIndices[index++] = nPointsLongitude + longitudeIndex;
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude - 1; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % nPointsLongitude;
         triangleIndices[index++] = (nPointsLatitude - 1) * nPointsLongitude + longitudeIndex;
         triangleIndices[index++] = (nPointsLatitude - 2) * nPointsLongitude + longitudeIndex;
         triangleIndices[index++] = (nPointsLatitude - 2) * nPointsLongitude + nextLongitudeIndex;
      }

      return new TriangleMesh3DDefinition("Ellipsoid Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Create a triangle mesh for the given polygon.
    *
    * @param description the description holding the polygon's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Polygon(Polygon2DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Polygon(null, description.getPolygonVertices(), description.isCounterClockwiseOrdered());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Create a triangle mesh for the given polygon.
    *
    * @param convexPolygon the polygon to create a mesh from.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Polygon(ConvexPolygon2DReadOnly convexPolygon)
   {
      return Polygon(null, convexPolygon);
   }

   /**
    * Create a triangle mesh for the given polygon.
    *
    * @param polygonPose   the 3D pose of the polygon. Can be {@code null}, in which case no transform
    *                      is applied.
    * @param convexPolygon the polygon to create a mesh from.
    * @return the generic triangle mesh or {@code null} if {@code convexPolygon} is {@code null}.
    */
   public static TriangleMesh3DDefinition Polygon(RigidBodyTransformReadOnly polygonPose, ConvexPolygon2DReadOnly convexPolygon)
   {
      if (convexPolygon == null)
         return null;
      else
         return Polygon(polygonPose, convexPolygon.getPolygonVerticesView(), !convexPolygon.isClockwiseOrdered());
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <p>
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b>
    * </p>
    *
    * @param polygonPose              the 3D pose of the polygon. Can be {@code null}, in which case no
    *                                 transform is applied.
    * @param cwOrderedPolygonVertices the clockwise-ordered vertices of the polygon.
    * @return the generic triangle mesh or {@code null} if {@code cwOrderedPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition PolygonClockwise(RigidBodyTransformReadOnly polygonPose, List<? extends Point2DReadOnly> cwOrderedPolygonVertices)
   {
      return Polygon(polygonPose, cwOrderedPolygonVertices, false);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <p>
    * <b> It is assumed that the polygon is convex and counter-clockwise ordered. </b>
    * </p>
    *
    * @param polygonPose               the 3D pose of the polygon. Can be {@code null}, in which case
    *                                  no transform is applied.
    * @param ccwOrderedPolygonVertices the counter-clockwise-ordered vertices of the polygon.
    * @return the generic triangle mesh or {@code null} if {@code ccwOrderedPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition PolygonCounterClockwise(RigidBodyTransformReadOnly polygonPose,
                                                                  List<? extends Point2DReadOnly> ccwOrderedPolygonVertices)
   {
      return Polygon(polygonPose, ccwOrderedPolygonVertices, true);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <p>
    * <b> It is assumed that the polygon is convex. </b>
    * </p>
    *
    * @param polygonPose             the 3D pose of the polygon. Can be {@code null}, in which case no
    *                                transform is applied.
    * @param polygonVertices         the counter-clockwise-ordered vertices of the polygon.
    * @param counterClockwiseOrdered {@code true} to indicate that the vertices are counter-clockwise
    *                                ordered, {@code false} for clockwise ordered.
    * @return the generic triangle mesh or {@code null} if {@code convexPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition Polygon(RigidBodyTransformReadOnly polygonPose, List<? extends Point2DReadOnly> polygonVertices,
                                                  boolean counterClockwiseOrdered)
   {
      if (polygonVertices == null)
         return null;

      int numberOfVertices = polygonVertices.size();

      if (numberOfVertices < 3)
         return null;

      Point3D32[] vertices = new Point3D32[numberOfVertices];
      Vector3D32[] normals = new Vector3D32[numberOfVertices];
      Point2D32[] texturePoints = new Point2D32[numberOfVertices];

      normals[0] = new Vector3D32(Axis3D.Z);
      if (polygonPose != null)
         polygonPose.transform(normals[0]);

      for (int i = 1; i < numberOfVertices; i++)
         normals[i] = new Vector3D32(normals[0]);

      float minX = polygonVertices.get(0).getX32();
      float minY = polygonVertices.get(0).getY32();
      float maxX = polygonVertices.get(0).getX32();
      float maxY = polygonVertices.get(0).getY32();

      for (int i = 1; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex2D;
         if (counterClockwiseOrdered)
            vertex2D = polygonVertices.get(i);
         else
            vertex2D = polygonVertices.get(numberOfVertices - 1 - i);

         if (vertex2D.getX32() > maxX)
            maxX = vertex2D.getX32();
         else if (vertex2D.getX32() < minX)
            minX = vertex2D.getX32();

         if (vertex2D.getY32() > maxY)
            maxY = vertex2D.getY32();
         else if (vertex2D.getY32() < minY)
            minY = vertex2D.getY32();
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex2D = polygonVertices.get(i);
         Point3D32 vertex3D = new Point3D32();
         vertex3D.set(vertex2D);
         if (polygonPose != null)
            polygonPose.transform(vertex3D);
         vertices[i] = vertex3D;

         float textureX = 1.0f - (vertex2D.getY32() - minY) / (maxY - minY);
         float textureY = 1.0f - (vertex2D.getX32() - minX) / (maxX - minX);
         texturePoints[i] = new Point2D32(textureX, textureY);
      }

      if (!counterClockwiseOrdered)
      {
         reverse(vertices);
         reverse(texturePoints);
      }

      int numberOfTriangles = numberOfVertices - 2;
      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;
      for (int j = 2; j < vertices.length; j++)
      {
         triangleIndices[index++] = 0;
         triangleIndices[index++] = j - 1;
         triangleIndices[index++] = j;
      }

      return new TriangleMesh3DDefinition("Polygon Factory", vertices, texturePoints, normals, triangleIndices);
   }

   /**
    * Create a triangle mesh for the given polygon.
    *
    * @param description the description holding the polygon's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Polygon(Polygon3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Polygon(description.getPolygonVertices(), description.isCounterClockwiseOrdered());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <p>
    * <b> It is assumed that the polygon is convex and clockwise ordered. </b>
    * </p>
    *
    * @param cwOrderedPolygonVertices the clockwise-ordered vertices of the polygon.
    * @return the generic triangle mesh or {@code null} if {@code cwOrderedPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition PolygonClockwise(List<? extends Point3DReadOnly> cwOrderedPolygonVertices)
   {
      return Polygon(cwOrderedPolygonVertices, false);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <p>
    * <b> It is assumed that the polygon is convex and counter-clockwise ordered. </b>
    * </p>
    *
    * @param ccwOrderedPolygonVertices the counter-clockwise-ordered vertices of the polygon.
    * @return the generic triangle mesh or {@code null} if {@code ccwOrderedPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition PolygonCounterClockwise(List<? extends Point3DReadOnly> ccwOrderedPolygonVertices)
   {
      return Polygon(ccwOrderedPolygonVertices, true);
   }

   /**
    * Create a triangle mesh for the given polygon.
    * <p>
    * <b> It is assumed that the polygon is convex. </b>
    * </p>
    *
    * @param convexPolygonVertices   the vertices of the polygon.
    * @param counterClockwiseOrdered {@code true} to indicate that the vertices are counter-clockwise
    *                                ordered, {@code false} for clockwise ordered.
    * @return the generic triangle mesh or {@code null} if {@code convexPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition Polygon(List<? extends Point3DReadOnly> convexPolygonVertices, boolean counterClockwiseOrdered)
   {
      if (convexPolygonVertices == null)
         return null;

      int numberOfVertices = convexPolygonVertices.size();

      if (numberOfVertices < 3)
         return null;

      int numberOfTriangles = numberOfVertices - 2;
      Vector3D32[] triangleNormals = new Vector3D32[numberOfTriangles];

      for (int i = 0; i < numberOfTriangles; i++)
      {
         Vector3D32 triangleNormal = new Vector3D32();
         EuclidGeometryTools.normal3DFromThreePoint3Ds(convexPolygonVertices.get(0),
                                                       convexPolygonVertices.get(i + 1),
                                                       convexPolygonVertices.get(i + 2),
                                                       triangleNormal);
         triangleNormals[i] = triangleNormal;
      }

      boolean areAllNormalsEqual = true;
      Vector3D32 polygonNormal = new Vector3D32();

      for (int i = 1; i < numberOfTriangles; i++)
      {
         if (areAllNormalsEqual && !triangleNormals[i - 1].epsilonEquals(triangleNormals[i], 1.0e-7))
         {
            areAllNormalsEqual = false;
         }

         polygonNormal.add(triangleNormals[i]);
      }

      polygonNormal.normalize();
      RotationMatrix polygonOrientation = new RotationMatrix();
      EuclidGeometryTools.orientation3DFromZUpToVector3D(polygonNormal, polygonOrientation);
      Point2D32[] texturePoints = new Point2D32[numberOfVertices];

      float minX = Float.POSITIVE_INFINITY;
      float minY = Float.POSITIVE_INFINITY;
      float maxX = Float.NEGATIVE_INFINITY;
      float maxY = Float.NEGATIVE_INFINITY;
      Point3D32 point = new Point3D32();

      for (int i = 0; i < numberOfVertices; i++)
      {
         polygonOrientation.inverseTransform(convexPolygonVertices.get(i), point);
         Point2D32 texturePoint = new Point2D32();
         texturePoint.set(-point.getY(), -point.getX());
         texturePoints[i] = texturePoint;
         minX = Math.min(minX, texturePoint.getX32());
         minY = Math.min(minY, texturePoint.getY32());
         maxX = Math.max(maxX, texturePoint.getX32());
         maxY = Math.max(maxY, texturePoint.getY32());
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         texturePoints[i].sub(minX, minY);
         texturePoints[i].scale(1.0 / (maxX - minX), 1.0 / (maxY - minY));
      }

      Point3D32[] vertices = convexPolygonVertices.stream().map(Point3D32::new).toArray(Point3D32[]::new);
      Vector3D32[] normals = new Vector3D32[numberOfVertices];
      int[] triangleIndices = new int[3 * numberOfTriangles];

      if (areAllNormalsEqual)
      {
         for (int i = 0; i < numberOfVertices; i++)
         {
            if (i < numberOfTriangles)
               normals[i] = triangleNormals[i];
            else
               normals[i] = new Vector3D32(triangleNormals[0]);
         }
      }
      else
      {
         normals[0] = polygonNormal;
         normals[1] = triangleNormals[0];

         for (int i = 1; i < numberOfVertices - 2; i++)
         {
            Vector3D32 normal = new Vector3D32();
            normal.interpolate(triangleNormals[i - 1], triangleNormals[i], 0.5);
            normal.normalize();
            normals[i] = normal;
         }
         Vector3D32 normal = new Vector3D32();
         normal.interpolate(triangleNormals[numberOfTriangles - 2], triangleNormals[numberOfTriangles - 1], 0.5);
         normal.normalize();
         normals[numberOfVertices - 2] = triangleNormals[numberOfTriangles - 1];
         normals[numberOfVertices - 1] = triangleNormals[numberOfTriangles - 1];
      }

      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int index = 0;
      for (int j = 2; j < numberOfVertices; j++)
      {
         triangleIndices[index++] = 0;
         triangleIndices[index++] = j - 1;
         triangleIndices[index++] = j;
      }

      return new TriangleMesh3DDefinition("Polygon Factory", vertices, texturePoints, normals, triangleIndices);
   }

   /**
    * Create a triangle mesh for the given polygon 2D and extrude it along the z-axis.
    *
    * @param description the description holding the polygon's properties.
    * @return the generic triangle mesh or {@code null} if {@code convexPolygon} is {@code null}.
    */
   public static TriangleMesh3DDefinition ExtrudedPolygon(ExtrudedPolygon2DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = ExtrudedPolygon(description.getPolygonVertices(),
                                                                description.isCounterClockwiseOrdered(),
                                                                description.getTopZ(),
                                                                description.getBottomZ());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Create a triangle mesh for the given polygon 2D and extrude it along the z-axis.
    *
    * @param convexPolygon   the polygon to create a mesh from.
    * @param extrusionHeight thickness of the extrusion. If {@code extrusionHeight < 0}, the polygon is
    *                        extruded toward z negative.
    * @return the generic triangle mesh or {@code null} if {@code convexPolygon} is {@code null}.
    */
   public static TriangleMesh3DDefinition ExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon, double extrusionHeight)
   {
      if (convexPolygon == null)
         return null;
      return ExtrudedPolygon(convexPolygon.getPolygonVerticesView(), false, extrusionHeight, 0.0);
   }

   /**
    * Create a triangle mesh for the given polygon 2D and extrude it along the z-axis.
    * <p>
    * <b> It is assumed that the polygon is convex and counter-clockwise ordered. </b>
    * </p>
    *
    * @param ccwOrderedPolygonVertices the counter-clockwise-ordered vertices of the polygon.
    * @param extrusionHeight           thickness of the extrusion. The polygon is extruded upward along
    *                                  the z-axis.
    * @return the generic triangle mesh or {@code null} if {@code ccwOrderedPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition ExtrudedPolygon(List<? extends Point2DReadOnly> ccwOrderedPolygonVertices, double extrusionHeight)
   {
      return ExtrudedPolygonCounterClockwise(ccwOrderedPolygonVertices, extrusionHeight, 0.0);
   }

   /**
    * Create a triangle mesh for the given polygon 2D and extrude it along the z-axis.
    * <p>
    * <b> It is assumed that the polygon is convex and counter-clockwise ordered. </b>
    * </p>
    *
    * @param ccwOrderedPolygonVertices the counter-clockwise-ordered vertices of the polygon.
    * @param topZ                      thickness of the extrusion. If {@code extrusionHeight < 0}, the
    *                                  polygon is extruded toward z negative.
    * @param bottomZ                   offset along the z-axis that is applied on all the vertices of
    *                                  the resulting mesh.
    * @return the generic triangle mesh or {@code null} if {@code ccwOrderedPolygonVertices} is
    *         {@code null}.
    */
   public static TriangleMesh3DDefinition ExtrudedPolygonCounterClockwise(List<? extends Point2DReadOnly> ccwOrderedPolygonVertices, double topZ,
                                                                          double bottomZ)
   {
      return ExtrudedPolygon(ccwOrderedPolygonVertices, true, topZ, bottomZ);
   }

   /**
    * Create a triangle mesh for the given polygon and extrude it along the z-axis.
    * <p>
    * <b> It is assumed that the polygon is convex and counter-clockwise ordered. </b>
    * </p>
    *
    * @param polygonVertices         the counter-clockwise-ordered vertices of the polygon.
    * @param counterClockwiseOrdered {@code true} to indicate that the vertices are counter-clockwise
    *                                ordered, {@code false} for clockwise ordered.
    * @param topZ                    z-coordinate of the bottom face.
    * @param bottomZ                 z-coordinate of the top face.
    * @return the generic triangle mesh or {@code null} if {@code polygonVertices} is {@code null}.
    */
   public static TriangleMesh3DDefinition ExtrudedPolygon(List<? extends Point2DReadOnly> polygonVertices, boolean counterClockwiseOrdered, double topZ,
                                                          double bottomZ)
   {
      if (polygonVertices == null || polygonVertices.size() < 3)
         return null;

      int numberOfVertices = polygonVertices.size();

      Point3D32 vertices[] = new Point3D32[6 * numberOfVertices + 4];
      Vector3D32 normals[] = new Vector3D32[6 * numberOfVertices + 4];
      Point2D32[] texturePoints = new Point2D32[6 * numberOfVertices + 4];

      float minX = polygonVertices.get(0).getX32();
      float minY = polygonVertices.get(0).getY32();
      float maxX = polygonVertices.get(0).getX32();
      float maxY = polygonVertices.get(0).getY32();

      for (int i = 1; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex = polygonVertices.get(i);

         if (vertex.getX32() > maxX)
            maxX = vertex.getX32();
         else if (vertex.getX32() < minX)
            minX = vertex.getX32();

         if (vertex.getY32() > maxY)
            maxY = vertex.getY32();
         else if (vertex.getY32() < minY)
            minY = vertex.getY32();
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex;

         if (counterClockwiseOrdered)
            vertex = polygonVertices.get(i);
         else
            vertex = polygonVertices.get(numberOfVertices - 1 - i);

         float vertexX = vertex.getX32();
         float vertexY = vertex.getY32();

         // Vertices for bottom face
         vertices[i] = new Point3D32(vertexX, vertexY, (float) bottomZ);
         normals[i] = new Vector3D32(0.0f, 0.0f, -1.0f);
         texturePoints[i] = new Point2D32(0.5f - 0.5f * (vertexY - minY) / (maxY - minY) + 0.5f, 0.5f - 0.5f * (vertexX - minX) / (maxX - minX));

         // Vertices for top face
         vertices[i + numberOfVertices] = new Point3D32(vertexX, vertexY, (float) (topZ));
         normals[i + numberOfVertices] = new Vector3D32(0.0f, 0.0f, 1.0f);
         texturePoints[i + numberOfVertices] = new Point2D32(0.5f - 0.5f * (vertexY - minY) / (maxY - minY), 0.5f - 0.5f * (vertexX - minX) / (maxX - minX));
      }

      double perimeter = 0.0;

      for (int i = 0; i < polygonVertices.size(); i++)
      {
         Point2DReadOnly vertex = polygonVertices.get(i);
         Point2DReadOnly nextVertex = polygonVertices.get((i + 1) % numberOfVertices);
         perimeter += vertex.distance(nextVertex);
      }

      double distanceAlongPerimeter = 0.0;
      double nextDistanceAlongPerimeter = 0.0;
      int nextIndex = counterClockwiseOrdered ? 0 : numberOfVertices - 1;
      Point2DReadOnly vertex;
      Point2DReadOnly nextVertex = polygonVertices.get(nextIndex);

      for (int i = 0; i < numberOfVertices + 1; i++)
      {
         vertex = nextVertex;

         if (counterClockwiseOrdered)
         {
            nextIndex++;
            nextIndex %= numberOfVertices;
         }
         else
         {
            nextIndex--;
            if (nextIndex < 0)
               nextIndex = numberOfVertices - 1;
         }
         nextVertex = polygonVertices.get(nextIndex);
         nextDistanceAlongPerimeter += nextVertex.distance(vertex);

         float vertexX = vertex.getX32();
         float vertexY = vertex.getY32();

         Point3D32 vertexBottom = new Point3D32(vertexX, vertexY, (float) bottomZ);
         Point3D32 vertexTop = new Point3D32(vertexX, vertexY, (float) (topZ));

         Point3D32 nextVertexBottom = new Point3D32(nextVertex.getX32(), nextVertex.getY32(), (float) bottomZ);
         Point3D32 nextVertexTop = new Point3D32(nextVertex.getX32(), nextVertex.getY32(), (float) (topZ));

         Vector3D normal = new Vector3D();
         normal.sub(nextVertexTop, vertexTop);
         normal.cross(Axis3D.Z, normal);
         normal.negate();
         normal.normalize();

         int vertexBottomIndex = 2 * i + 2 * numberOfVertices;
         int nextVertexBottomIndex = vertexBottomIndex + 1;
         int vertexTopIndex = vertexBottomIndex + 2 * (numberOfVertices + 1);
         int nextVertexTopIndex = vertexTopIndex + 1;

         // Vertices for side faces
         // Bottom
         vertices[vertexBottomIndex] = vertexBottom;
         normals[vertexBottomIndex] = new Vector3D32(normal);
         texturePoints[vertexBottomIndex] = new Point2D32((float) (distanceAlongPerimeter / perimeter), 1.0f);

         vertices[nextVertexBottomIndex] = nextVertexBottom;
         normals[nextVertexBottomIndex] = new Vector3D32(normal);
         texturePoints[nextVertexBottomIndex] = new Point2D32((float) (nextDistanceAlongPerimeter / perimeter), 1.0f);

         // Top
         vertices[vertexTopIndex] = vertexTop;
         normals[vertexTopIndex] = new Vector3D32(normal);
         texturePoints[vertexTopIndex] = new Point2D32((float) (distanceAlongPerimeter / perimeter), 0.5f);

         vertices[nextVertexTopIndex] = nextVertexTop;
         normals[nextVertexTopIndex] = new Vector3D32(normal);
         texturePoints[nextVertexTopIndex] = new Point2D32((float) (nextDistanceAlongPerimeter / perimeter), 0.5f);

         distanceAlongPerimeter = nextDistanceAlongPerimeter;
      }

      int numberOfTriangles = 4 * numberOfVertices;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      for (int i = 1; i < numberOfVertices; i++)
      {
         // Bottom face
         triangleIndices[index++] = (i + 1) % numberOfVertices;
         triangleIndices[index++] = i;
         triangleIndices[index++] = 0;

         // Top face
         triangleIndices[index++] = numberOfVertices;
         triangleIndices[index++] = i + numberOfVertices;
         triangleIndices[index++] = (i + 1) % numberOfVertices + numberOfVertices;
      }

      // Side faces
      for (int i = 0; i < numberOfVertices + 1; i++)
      {
         // Lower triangle
         triangleIndices[index++] = 2 * i + 2 * numberOfVertices;
         triangleIndices[index++] = (2 * i + 1) % (2 * numberOfVertices + 2) + 2 * numberOfVertices;
         triangleIndices[index++] = 2 * i + 4 * numberOfVertices + 2;
         // Upper triangle
         triangleIndices[index++] = (2 * i + 1) % (2 * numberOfVertices + 2) + 2 * numberOfVertices;
         triangleIndices[index++] = (2 * i + 1) % (2 * numberOfVertices + 2) + 4 * numberOfVertices + 2;
         triangleIndices[index++] = 2 * i + 4 * numberOfVertices + 2;
      }

      return new TriangleMesh3DDefinition("ExtrudedPolygon Factory", vertices, texturePoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D hemi-ellipsoid, i.e. top half of an ellipsoid with the bottom
    * closed by a flat cap.
    *
    * @param description the description holding the hemi-ellipsoid's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition HemiEllipsoid(HemiEllipsoid3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = HemiEllipsoid(description.getRadiusX(),
                                                              description.getRadiusY(),
                                                              description.getRadiusZ(),
                                                              description.getResolution(),
                                                              description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D hemi-ellipsoid, i.e. top half of an ellipsoid with the bottom
    * closed by a flat cap.
    *
    * @param radiusX             radius of the ellipsoid along the x-axis.
    * @param radiusY             radius of the ellipsoid along the y-axis.
    * @param radiusZ             radius of the ellipsoid along the z-axis.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition HemiEllipsoid(double radiusX, double radiusY, double radiusZ, int latitudeResolution, int longitudeResolution)
   {
      return HemiEllipsoid((float) radiusX, (float) radiusY, (float) radiusZ, latitudeResolution, longitudeResolution);
   }

   /**
    * Creates a triangle mesh for a 3D hemi-ellipsoid, i.e. top half of an ellipsoid with the bottom
    * closed by a flat cap.
    *
    * @param radiusX             radius of the ellipsoid along the x-axis.
    * @param radiusY             radius of the ellipsoid along the y-axis.
    * @param radiusZ             radius of the ellipsoid along the z-axis.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition HemiEllipsoid(float radiusX, float radiusY, float radiusZ, int latitudeResolution, int longitudeResolution)
   {
      if (longitudeResolution % 2 == 1)
         longitudeResolution += 1;
      int nPointsLongitude = longitudeResolution + 1;
      int nPointsLatitude = latitudeResolution + 1;

      // Reminder of longitude and latitude: http://www.geographyalltheway.com/ks3_geography/maps_atlases/longitude_latitude.htm
      Point3D32 points[] = new Point3D32[(nPointsLatitude + 1) * nPointsLongitude];
      Vector3D32[] normals = new Vector3D32[(nPointsLatitude + 1) * nPointsLongitude];
      Point2D32 textPoints[] = new Point2D32[(nPointsLatitude + 1) * nPointsLongitude];

      for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude; longitudeIndex++)
      {
         float longitudeAngle = TwoPi * ((float) longitudeIndex / (float) (nPointsLongitude - 1));
         float cosLongitude = (float) Math.cos(longitudeAngle);
         float sinLongitude = (float) Math.sin(longitudeAngle);
         float textureX = (float) longitudeIndex / (float) (nPointsLongitude - 1);

         for (int latitudeIndex = 1; latitudeIndex < nPointsLatitude - 1; latitudeIndex++)
         {
            float latitudeAngle = HalfPi * ((float) (latitudeIndex - 1) / (float) (nPointsLatitude - 2));
            float cosLatitude = (float) Math.cos(latitudeAngle);
            float sinLatitude = (float) Math.sin(latitudeAngle);

            int currentIndex = (latitudeIndex + 1) * nPointsLongitude + longitudeIndex;
            float normalX = cosLongitude * cosLatitude;
            float normalY = sinLongitude * cosLatitude;
            float normalZ = sinLatitude;
            float vertexX = radiusX * normalX;
            float vertexY = radiusY * normalY;
            float vertexZ = radiusZ * normalZ;
            points[currentIndex] = new Point3D32(vertexX, vertexY, vertexZ);
            normals[currentIndex] = new Vector3D32(normalX, normalY, normalZ);

            float textureY = 0.5f * (1.0f - sinLatitude);
            textPoints[currentIndex] = new Point2D32(textureX, textureY);
         }

         // Bottom side
         int currentIndex = nPointsLongitude + longitudeIndex;
         float vertexX = (float) (radiusX * Math.cos(longitudeAngle));
         float vertexY = (float) (radiusY * Math.sin(longitudeAngle));
         float vertexZ = 0.0f;
         points[currentIndex] = new Point3D32(vertexX, vertexY, vertexZ);
         normals[currentIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[currentIndex] = new Point2D32(textureX, 0.5f);

         textureX += 0.5f / (nPointsLongitude - 1);

         // Bottom center
         int southPoleIndex = longitudeIndex;
         points[southPoleIndex] = new Point3D32(0.0f, 0.0f, 0.0f);
         normals[southPoleIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[southPoleIndex] = new Point2D32(textureX, 1.0f - 1.0f / 256.0f);

         // North pole
         int northPoleIndex = nPointsLatitude * nPointsLongitude + longitudeIndex;
         points[northPoleIndex] = new Point3D32(0.0f, 0.0f, radiusZ);
         normals[northPoleIndex] = new Vector3D32(0.0f, 0.0f, 1.0f);
         textPoints[northPoleIndex] = new Point2D32(textureX, 1.0f / 256.0f);
      }

      int numberOfTriangles = 2 * (nPointsLatitude - 1) * (nPointsLongitude - 1);
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Mid-latitude faces
      for (int latitudeIndex = 1; latitudeIndex < nPointsLatitude - 1; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude - 1; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % nPointsLongitude;
            int nextLatitudeIndex = latitudeIndex + 1;

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * nPointsLongitude + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * nPointsLongitude + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * nPointsLongitude + longitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * nPointsLongitude + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * nPointsLongitude + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * nPointsLongitude + longitudeIndex;
         }
      }

      // Bottom face
      for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude - 1; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % nPointsLongitude;
         triangleIndices[index++] = longitudeIndex;
         triangleIndices[index++] = nPointsLongitude + nextLongitudeIndex;
         triangleIndices[index++] = nPointsLongitude + longitudeIndex;
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < nPointsLongitude - 1; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % nPointsLongitude;
         triangleIndices[index++] = (nPointsLatitude - 0) * nPointsLongitude + longitudeIndex;
         triangleIndices[index++] = (nPointsLatitude - 1) * nPointsLongitude + longitudeIndex;
         triangleIndices[index++] = (nPointsLatitude - 1) * nPointsLongitude + nextLongitudeIndex;
      }

      return new TriangleMesh3DDefinition("HemiEllipsoid Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D cylinder.
    * <p>
    * The cylinder's axis is aligned with the z-axis. When {@code centered} is true, the cylinder is
    * centered at the origin, when false its bottom face is centered at the origin.
    * </p>
    *
    * @param description the description holding the cylinder's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Cylinder(Cylinder3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Cylinder(description.getRadius(),
                                                         description.getLength(),
                                                         description.getResolution(),
                                                         description.isCentered());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D cylinder.
    * <p>
    * The cylinder's axis is aligned with the z-axis. When {@code centered} is true, the cylinder is
    * centered at the origin, when false its bottom face is centered at the origin.
    * </p>
    *
    * @param radius     the cylinder's radius.
    * @param height     the cylinder's height or length.
    * @param resolution the resolution for the cylindrical part.
    * @param centered   {@code true} to center the cylinder at the origin, {@code false} to center its
    *                   bottom face at the origin.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Cylinder(double radius, double height, int resolution, boolean centered)
   {
      return Cylinder((float) radius, (float) height, resolution, centered);
   }

   /**
    * Creates a triangle mesh for a 3D cylinder.
    * <p>
    * The cylinder's axis is aligned with the z-axis. When {@code centered} is true, the cylinder is
    * centered at the origin, when false its bottom face is centered at the origin.
    * </p>
    * 
    * @param radius     the cylinder's radius.
    * @param height     the cylinder's height or length.
    * @param resolution the resolution for the cylindrical part.
    * @param centered   {@code true} to center the cylinder at the origin, {@code false} to center its
    *                   bottom face at the origin.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Cylinder(float radius, float height, int resolution, boolean centered)
   {
      Point3D32 points[] = new Point3D32[4 * resolution + 2];
      Vector3D32 normals[] = new Vector3D32[4 * resolution + 2];
      Point2D32 texturePoints[] = new Point2D32[4 * resolution + 2];

      float zTop = centered ? 0.5f * height : height;
      float zBottom = centered ? -0.5f * height : 0.0f;

      for (int i = 0; i < resolution; i++)
      {
         double angle = i * TwoPi / (resolution - 1.0);
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float vertexX = radius * cosAngle;
         float vertexY = radius * sinAngle;

         // Bottom vertices
         points[i] = new Point3D32(vertexX, vertexY, zBottom);
         normals[i] = new Vector3D32(0.0f, 0.0f, -1.0f);
         texturePoints[i] = new Point2D32(0.25f * (1f + sinAngle) + 0.5f, 0.25f * (1f - cosAngle));

         // Top vertices
         points[i + resolution] = new Point3D32(vertexX, vertexY, zTop);
         normals[i + resolution] = new Vector3D32(0.0f, 0.0f, 1.0f);
         texturePoints[i + resolution] = new Point2D32(0.25f * (1f - sinAngle), 0.25f * (1f - cosAngle));

         // Outer vertices
         // Bottom
         points[i + 2 * resolution] = new Point3D32(vertexX, vertexY, zBottom);
         normals[i + 2 * resolution] = new Vector3D32(cosAngle, sinAngle, 0.0f);
         texturePoints[i + 2 * resolution] = new Point2D32(i / (resolution - 1.0f), 1.0f);

         // Top
         points[i + 3 * resolution] = new Point3D32(vertexX, vertexY, zTop);
         normals[i + 3 * resolution] = new Vector3D32(cosAngle, sinAngle, 0.0f);
         texturePoints[i + 3 * resolution] = new Point2D32(i / (resolution - 1.0f), 0.5f);
      }

      // Center of bottom cap
      points[4 * resolution] = new Point3D32(0.0f, 0.0f, zBottom);
      normals[4 * resolution] = new Vector3D32(0.0f, 0.0f, -1.0f);
      texturePoints[4 * resolution] = new Point2D32(0.75f, 0.25f);
      // Center of top cap
      points[4 * resolution + 1] = new Point3D32(0.0f, 0.0f, zTop);
      normals[4 * resolution + 1] = new Vector3D32(0.0f, 0.0f, 1.0f);
      texturePoints[4 * resolution + 1] = new Point2D32(0.25f, 0.25f);

      int numberOfTriangles = 4 * resolution;
      int[] triangleIndices = new int[6 * numberOfTriangles];

      int index = 0;

      for (int i = 0; i < resolution; i++)
      { // The bottom cap
         triangleIndices[index++] = (i + 1) % resolution;
         triangleIndices[index++] = i;
         triangleIndices[index++] = 4 * resolution;
      }

      for (int i = 0; i < resolution; i++)
      { // The top cap
         triangleIndices[index++] = 4 * resolution + 1;
         triangleIndices[index++] = i + resolution;
         triangleIndices[index++] = (i + 1) % resolution + resolution;
      }

      for (int i = 0; i < resolution; i++)
      { // The cylinder part
        // Lower triangle
         triangleIndices[index++] = i + 2 * resolution;
         triangleIndices[index++] = (i + 1) % resolution + 2 * resolution;
         triangleIndices[index++] = i + 3 * resolution;
         // Upper triangle
         triangleIndices[index++] = (i + 1) % resolution + 2 * resolution;
         triangleIndices[index++] = (i + 1) % resolution + 3 * resolution;
         triangleIndices[index++] = i + 3 * resolution;
      }

      return new TriangleMesh3DDefinition("Cylinder Factory", points, texturePoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D cone.
    * <p>
    * The cone's axis is aligned with the z-axis and is positioned such that the center of its bottom
    * face is at the origin.
    * </p>
    * 
    * @param description the description holding the cone's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Cone(Cone3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Cone(description.getHeight(), description.getRadius(), description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D cone.
    * <p>
    * The cone's axis is aligned with the z-axis and is positioned such that the center of its bottom
    * face is at the origin.
    * </p>
    * 
    * @param height     the height of the cone.
    * @param radius     the radius of the base.
    * @param resolution the resolution for the cylindrical part of the cone.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Cone(double height, double radius, int resolution)
   {
      return Cone((float) height, (float) radius, resolution);
   }

   /**
    * Creates a triangle mesh for a 3D cone.
    * <p>
    * The cone's axis is aligned with the z-axis and is positioned such that the center of its bottom
    * face is at the origin.
    * </p>
    * 
    * @param height     the height of the cone.
    * @param radius     the radius of the base.
    * @param resolution the resolution for the cylindrical part of the cone.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Cone(float height, float radius, int resolution)
   {
      Point3D32[] vertices = new Point3D32[3 * resolution + 1];
      Vector3D32[] normals = new Vector3D32[3 * resolution + 1];
      Point2D32[] texturePoints = new Point2D32[3 * resolution + 1];

      // This is equal to half of the opening angle of the cone at its top. Used to compute the normals.
      float slopeAngle = (float) Math.atan2(radius, height);
      float cosSlopeAngle = (float) Math.cos(slopeAngle);
      float sinSlopeAngle = (float) Math.sin(slopeAngle);

      for (int i = 0; i < resolution; i++)
      {
         double angle = i * TwoPi / (resolution - 1);
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float vertexX = radius * cosAngle;
         float vertexY = radius * sinAngle;

         // Vertices for the bottom part.
         vertices[i] = new Point3D32(vertexX, vertexY, 0.0f);
         normals[i] = new Vector3D32(0.0f, 0.0f, -1.0f);
         texturePoints[i] = new Point2D32(0.25f * (1f + sinAngle), 0.25f * (1f - cosAngle));

         // Vertices for the side part.
         vertices[i + resolution] = new Point3D32(vertexX, vertexY, 0.0f);
         normals[i + resolution] = new Vector3D32(cosSlopeAngle * cosAngle, cosSlopeAngle * sinAngle, sinSlopeAngle);
         texturePoints[i + resolution] = new Point2D32(i / (resolution - 1.0f), 1.0f);
         vertices[i + 2 * resolution] = new Point3D32(0.0f, 0.0f, height);
         normals[i + 2 * resolution] = new Vector3D32(cosSlopeAngle * cosAngle, cosSlopeAngle * sinAngle, sinSlopeAngle);
         texturePoints[i + 2 * resolution] = new Point2D32(i / (resolution - 1.0f), 0.5f);
      }

      // The center of the bottom
      int bottomCenterIndex = 3 * resolution;
      vertices[bottomCenterIndex] = new Point3D32(0.0f, 0.0f, 0.0f);
      normals[bottomCenterIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
      texturePoints[bottomCenterIndex] = new Point2D32(0.25f, 0.25f);

      int numberOfTriangles = 2 * resolution;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      for (int i = 0; i < resolution; i++)
      {
         // The bottom face
         triangleIndices[index++] = bottomCenterIndex;
         triangleIndices[index++] = (i + 1) % resolution;
         triangleIndices[index++] = i;

         // The side faces
         triangleIndices[index++] = i + resolution;
         triangleIndices[index++] = (i + 1) % resolution + resolution;
         triangleIndices[index++] = i + 2 * resolution;
      }

      return new TriangleMesh3DDefinition("Cone Factory", vertices, texturePoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D truncated cone which base and top are ellipses.
    * <p>
    * The cone's axis is aligned with the z-axis and is positioned such that the center of its bottom
    * face is at the origin.
    * </p>
    * 
    * @param description the description holding the cone's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition TruncatedCone(TruncatedCone3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = TruncatedCone(description.getHeight(),
                                                              description.getBaseRadiusX(),
                                                              description.getBaseRadiusY(),
                                                              description.getTopRadiusX(),
                                                              description.getTopRadiusY(),
                                                              description.getResolution(),
                                                              description.getCentered());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D truncated cone which base and top are ellipses.
    * <p>
    * The cone's axis is aligned with the z-axis and is positioned such that the center of its bottom
    * face is at the origin.
    * </p>
    * 
    * @param height      the cone's height.
    * @param baseRadiusX radius around the x-axis of the base ellipse.
    * @param baseRadiusY radius around the y-axis of the base ellipse.
    * @param topRadiusX  radius around the x-axis of the top ellipse.
    * @param topRadiusY  radius around the x-axis of the top ellipse.
    * @param resolution  the resolution for the cylindrical part of the cone.
    * @param centered    {@code true} to center the truncated cone at the origin, {@code false} to
    *                    center its bottom face at the origin.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition TruncatedCone(double height, double baseRadiusX, double baseRadiusY, double topRadiusX, double topRadiusY,
                                                        int resolution, boolean centered)
   {
      return TruncatedCone((float) height, (float) baseRadiusX, (float) baseRadiusY, (float) topRadiusX, (float) topRadiusY, resolution, centered);
   }

   /**
    * Creates a triangle mesh for a 3D truncated cone which base and top are ellipses.
    * <p>
    * The cone's axis is aligned with the z-axis and is positioned such that the center of its bottom
    * face is at the origin.
    * </p>
    * 
    * @param height      the cone's height.
    * @param baseRadiusX radius around the x-axis of the base ellipse.
    * @param baseRadiusY radius around the y-axis of the base ellipse.
    * @param topRadiusX  radius around the x-axis of the top ellipse.
    * @param topRadiusY  radius around the x-axis of the top ellipse.
    * @param resolution  the resolution for the cylindrical part of the cone.
    * @param centered    {@code true} to center the truncated cone at the origin, {@code false} to
    *                    center its bottom face at the origin.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition TruncatedCone(float height, float baseRadiusX, float baseRadiusY, float topRadiusX, float topRadiusY, int resolution,
                                                        boolean centered)
   {
      Point3D32 points[] = new Point3D32[4 * resolution + 2];
      Vector3D32[] normals = new Vector3D32[4 * resolution + 2];
      Point2D32[] textPoints = new Point2D32[4 * resolution + 2];

      float topZ = centered ? 0.5f * height : height;
      float bottomZ = centered ? -0.5f * height : 0.0f;

      for (int i = 0; i < resolution; i++)
      {
         double angle = i * TwoPi / (resolution - 1);
         float cosAngle = (float) Math.cos(angle);
         float sinAngle = (float) Math.sin(angle);

         float baseX = baseRadiusX * cosAngle;
         float baseY = baseRadiusY * sinAngle;
         float topX = topRadiusX * cosAngle;
         float topY = topRadiusY * sinAngle;

         // Bottom face vertices
         points[i] = new Point3D32(baseX, baseY, bottomZ);
         normals[i] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[i] = new Point2D32(0.25f * (1f + sinAngle) + 0.5f, 0.25f * (1f - cosAngle));

         // Top face vertices
         points[i + resolution] = new Point3D32(topX, topY, topZ);
         normals[i + resolution] = new Vector3D32(0.0f, 0.0f, 1.0f);
         textPoints[i + resolution] = new Point2D32(0.25f * (1f - sinAngle), 0.25f * (1f - cosAngle));

         // Cone face
         float currentBaseRadius = (float) Math.sqrt(baseX * baseX + baseY * baseY);
         float currentTopRadius = (float) Math.sqrt(topX * topX + topY * topY);
         float openingAngle = (float) Math.atan((currentBaseRadius - currentTopRadius) / height);
         float baseAngle = (float) Math.atan2(baseY, baseX);
         float topAngle = (float) Math.atan2(topY, topX);
         points[i + 2 * resolution] = new Point3D32(baseX, baseY, bottomZ);
         normals[i + 2 * resolution] = new Vector3D32((float) (Math.cos(baseAngle) * Math.cos(openingAngle)),
                                                      (float) (Math.sin(baseAngle) * Math.cos(openingAngle)),
                                                      (float) Math.sin(openingAngle));
         textPoints[i + 2 * resolution] = new Point2D32(i / (resolution - 1.0f), 1.0f);
         points[i + 3 * resolution] = new Point3D32(topX, topY, topZ);
         normals[i + 3 * resolution] = new Vector3D32((float) (Math.cos(topAngle) * Math.cos(openingAngle)),
                                                      (float) (Math.sin(topAngle) * Math.cos(openingAngle)),
                                                      (float) Math.sin(openingAngle));
         textPoints[i + 3 * resolution] = new Point2D32(i / (resolution - 1.0f), 0.5f);
      }

      // Bottom center
      points[4 * resolution] = new Point3D32(0.0f, 0.0f, bottomZ);
      normals[4 * resolution] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[4 * resolution] = new Point2D32(0.75f, 0.25f);
      // Top center
      points[4 * resolution + 1] = new Point3D32(0.0f, 0.0f, topZ);
      normals[4 * resolution + 1] = new Vector3D32(0.0f, 0.0f, 1.0f);
      textPoints[4 * resolution + 1] = new Point2D32(0.25f, 0.25f);

      int numberOfTriangles = 4 * resolution;
      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;

      for (int i = 0; i < resolution; i++)
      {
         // Bottom face
         triangleIndices[index++] = 4 * resolution;
         triangleIndices[index++] = (i + 1) % resolution;
         triangleIndices[index++] = i;
         // Top face
         triangleIndices[index++] = 4 * resolution + 1;
         triangleIndices[index++] = i + resolution;
         triangleIndices[index++] = (i + 1) % resolution + resolution;
         //Cone face: lower triangle
         triangleIndices[index++] = i + 2 * resolution;
         triangleIndices[index++] = (i + 1) % resolution + 2 * resolution;
         triangleIndices[index++] = i + 3 * resolution;
         //Cone face: upper triangle
         triangleIndices[index++] = (i + 1) % resolution + 2 * resolution;
         triangleIndices[index++] = (i + 1) % resolution + 3 * resolution;
         triangleIndices[index++] = i + 3 * resolution;
      }

      return new TriangleMesh3DDefinition("TruncatedCone Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D torus.
    * <p>
    * The torus' axis is aligned with the z-axis and its centroid at the origin.
    * </p>
    * 
    * @param description the description holding the torus's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Torus(Torus3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Torus(description.getMajorRadius(), description.getMinorRadius(), description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D torus.
    * <p>
    * The torus' axis is aligned with the z-axis and its centroid at the origin.
    * </p>
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param resolution  used to define the longitudinal and radial resolutions.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Torus(double majorRadius, double minorRadius, int resolution)
   {
      return Torus((float) majorRadius, (float) minorRadius, resolution);
   }

   /**
    * Creates a triangle mesh for a 3D torus.
    * <p>
    * The torus' axis is aligned with the z-axis and its centroid at the origin.
    * </p>
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param resolution  used to define the longitudinal and radial resolutions.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Torus(float majorRadius, float minorRadius, int resolution)
   {
      return ArcTorus(0.0f, (float) (2.0 * Math.PI), majorRadius, minorRadius, resolution);
   }

   /**
    * Creates a triangle mesh for an open partial 3D torus.
    * <p>
    * The torus' axis is aligned with the z-axis and its centroid at the origin.
    * </p>
    * 
    * @param description the description holding the torus's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition ArcTorus(ArcTorus3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = ArcTorus(description.getStartAngle(),
                                                         description.getEndAngle(),
                                                         description.getMajorRadius(),
                                                         description.getMinorRadius(),
                                                         description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for an open partial 3D torus.
    * <p>
    * The torus' axis is aligned with the z-axis and its centroid at the origin.
    * </p>
    * 
    * @param startAngle  the angle at which the torus starts. The angle is in radians, it is expressed
    *                    with respect to the x-axis, and a positive angle corresponds to a
    *                    counter-clockwise rotation.
    * @param endAngle    the angle at which the torus ends. If {@code startAngle == endAngle} the torus
    *                    will be closed. The angle is in radians, it is expressed with respect to the
    *                    x-axis, and a positive angle corresponds to a counter-clockwise rotation.
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param resolution  used to define the longitudinal and radial resolutions.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition ArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, int resolution)
   {
      return ArcTorus((float) startAngle, (float) endAngle, (float) majorRadius, (float) minorRadius, resolution);
   }

   /**
    * Creates a triangle mesh for an open partial 3D torus.
    * <p>
    * The torus' axis is aligned with the z-axis and its centroid at the origin.
    * </p>
    * 
    * @param startAngle  the angle at which the torus starts. The angle is in radians, it is expressed
    *                    with respect to the x-axis, and a positive angle corresponds to a
    *                    counter-clockwise rotation.
    * @param endAngle    the angle at which the torus ends. If {@code startAngle == endAngle} the torus
    *                    will be closed. The angle is in radians, it is expressed with respect to the
    *                    x-axis, and a positive angle corresponds to a counter-clockwise rotation.
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param resolution  used to define the longitudinal and radial resolutions.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition ArcTorus(float startAngle, float endAngle, float majorRadius, float minorRadius, int resolution)
   {
      startAngle = (float) EuclidCoreTools.shiftAngleInRange(startAngle, 0.0);
      endAngle = (float) EuclidCoreTools.shiftAngleInRange(endAngle, 0.0);

      if (EuclidCoreTools.epsilonEquals(endAngle, 0.0, 1.0e-6))
         endAngle = TwoPi;

      float torusSpanAngle = endAngle - startAngle;
      boolean isClosed = MathTools.epsilonEquals(torusSpanAngle, TwoPi, 1.0e-3);

      // Make things a bit clearer.
      int majorN = resolution;
      // Make things a bit clearer.
      int minorN = resolution;

      float stepAngle = (endAngle - startAngle) / (majorN - 1);

      int numberOfVertices = isClosed ? majorN * minorN : majorN * minorN + 2 * (resolution + 1);
      Point3D32 points[] = new Point3D32[numberOfVertices];
      Vector3D32[] normals = new Vector3D32[numberOfVertices];
      Point2D32[] texturePoints = new Point2D32[numberOfVertices];

      float centerX, centerY;
      float pX, pY, pZ;
      float textureY, textureX;

      // Core part of the torus
      for (int majorIndex = 0; majorIndex < majorN; majorIndex++)
      {
         float majorAngle = startAngle + majorIndex * stepAngle;
         float cosMajorAngle = (float) Math.cos(majorAngle);
         float sinMajorAngle = (float) Math.sin(majorAngle);
         centerX = majorRadius * cosMajorAngle;
         centerY = majorRadius * sinMajorAngle;

         textureY = (float) majorIndex / (float) (majorN - 1);

         for (int minorIndex = 0; minorIndex < minorN; minorIndex++)
         {
            int currentIndex = majorIndex * minorN + minorIndex;
            float minorAngle = minorIndex * 2.0f * (float) Math.PI / (minorN - 1.0f);
            float cosMinorAngle = (float) Math.cos(minorAngle);
            float sinMinorAngle = (float) Math.sin(minorAngle);
            pX = centerX + minorRadius * cosMajorAngle * cosMinorAngle;
            pY = centerY + minorRadius * sinMajorAngle * cosMinorAngle;
            pZ = minorRadius * sinMinorAngle;
            points[currentIndex] = new Point3D32(pX, pY, pZ);
            normals[currentIndex] = new Vector3D32(cosMajorAngle * cosMinorAngle, sinMajorAngle * cosMinorAngle, sinMinorAngle);
            textureX = (float) minorIndex / (float) (minorN - 1);
            if (!isClosed)
               textureX *= 0.5f;
            texturePoints[currentIndex] = new Point2D32(textureX, textureY);
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
            points[currentIndex] = new Point3D32(pX, pY, pZ);
            normals[currentIndex] = new Vector3D32(sinStartAngle, -cosStartAngle, 0.0f);
            textureX = 0.75f + 0.25f * cosMinorAngle;
            textureY = 0.25f - 0.25f * sinMinorAngle;
            texturePoints[currentIndex] = new Point2D32(textureX, textureY);
         }

         // First end center
         int firstEndCenterIndex = numberOfVertices - 2;
         points[firstEndCenterIndex] = new Point3D32(centerX, centerY, 0.0f);
         normals[firstEndCenterIndex] = new Vector3D32(sinStartAngle, -cosStartAngle, 0.0f);
         texturePoints[firstEndCenterIndex] = new Point2D32(0.75f, 0.25f);

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
            points[currentIndex] = new Point3D32(pX, pY, pZ);
            normals[currentIndex] = new Vector3D32(-sinEndAngle, cosEndAngle, 0.0f);
            textureX = 0.75f - 0.25f * cosMinorAngle;
            textureY = 0.75f - 0.25f * sinMinorAngle;
            texturePoints[currentIndex] = new Point2D32(textureX, textureY);
         }

         // Second end center
         int secondEndCenterIndex = numberOfVertices - 1;
         points[secondEndCenterIndex] = new Point3D32(centerX, centerY, 0.0f);
         normals[secondEndCenterIndex] = new Vector3D32(-sinEndAngle, cosEndAngle, 0.0f);
         texturePoints[secondEndCenterIndex] = new Point2D32(0.75f, 0.75f);

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

      return new TriangleMesh3DDefinition("ArcTorus Factory", points, texturePoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D box of size ({@code sizeX}, {@code sizeY}, {@code sizeZ}).
    *
    * @param description the description holding the box's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Box(Box3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Box(description.getSizeX(), description.getSizeY(), description.getSizeZ(), description.isCentered());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D box of size ({@code sizeX}, {@code sizeY}, {@code sizeZ}).
    *
    * @param sizeX    box size along the x-axis.
    * @param sizeY    box size along the y-axis.
    * @param sizeZ    box size along the z-axis.
    * @param centered when {@code true} the generated mesh is centered at the origin, when
    *                 {@code false} the bottom face of the box is centered at the origin.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Box(double sizeX, double sizeY, double sizeZ, boolean centered)
   {
      return Box((float) sizeX, (float) sizeY, (float) sizeZ, centered);
   }

   /**
    * Creates a triangle mesh for a 3D box of size ({@code sizeX}, {@code sizeY}, {@code sizeZ}).
    *
    * @param sizeX    box size along the x-axis.
    * @param sizeY    box size along the y-axis.
    * @param sizeZ    box size along the z-axis.
    * @param centered when {@code true} the generated mesh is centered at the origin, when
    *                 {@code false} the bottom face of the box is centered at the origin.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Box(float sizeX, float sizeY, float sizeZ, boolean centered)
   {
      Point3D32 points[] = new Point3D32[24];
      Vector3D32[] normals = new Vector3D32[24];
      Point2D32 textPoints[] = new Point2D32[24];

      float zBottom = centered ? -sizeZ / 2f : 0;
      float zTop = centered ? sizeZ / 2f : sizeZ;

      // Bottom vertices for bottom face
      points[0] = new Point3D32(-sizeX / 2.0f, -sizeY / 2.0f, zBottom);
      normals[0] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[0] = new Point2D32(0.5f, 0.5f);
      points[1] = new Point3D32(sizeX / 2.0f, -sizeY / 2.0f, zBottom);
      normals[1] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[1] = new Point2D32(0.25f, 0.5f);
      points[2] = new Point3D32(sizeX / 2.0f, sizeY / 2.0f, zBottom);
      normals[2] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[2] = new Point2D32(0.25f, 0.25f);
      points[3] = new Point3D32(-sizeX / 2.0f, sizeY / 2.0f, zBottom);
      normals[3] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[3] = new Point2D32(0.5f, 0.25f);

      // Top vertices for top face
      points[4] = new Point3D32(-sizeX / 2.0f, -sizeY / 2.0f, zTop);
      normals[4] = new Vector3D32(0.0f, 0.0f, 1.0f);
      textPoints[4] = new Point2D32(0.75f, 0.5f);
      points[5] = new Point3D32(sizeX / 2.0f, -sizeY / 2.0f, zTop);
      normals[5] = new Vector3D32(0.0f, 0.0f, 1.0f);
      textPoints[5] = new Point2D32(1.0f, 0.5f);
      points[6] = new Point3D32(sizeX / 2.0f, sizeY / 2.0f, zTop);
      normals[6] = new Vector3D32(0.0f, 0.0f, 1.0f);
      textPoints[6] = new Point2D32(1.0f, 0.25f);
      points[7] = new Point3D32(-sizeX / 2.0f, sizeY / 2.0f, zTop);
      normals[7] = new Vector3D32(0.0f, 0.0f, 1.0f);
      textPoints[7] = new Point2D32(0.75f, 0.25f);

      // Left face vertices
      points[8] = new Point3D32(points[2]);
      normals[8] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[8] = new Point2D32(0.25f, 0.25f);
      points[9] = new Point3D32(points[3]);
      normals[9] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[9] = new Point2D32(0.5f, 0.25f);
      points[10] = new Point3D32(points[6]);
      normals[10] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[10] = new Point2D32(0.25f, 0.0f);
      points[11] = new Point3D32(points[7]);
      normals[11] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[11] = new Point2D32(0.5f, 0.0f);

      // Right face vertices
      points[12] = new Point3D32(points[0]);
      normals[12] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[12] = new Point2D32(0.5f, 0.5f);
      points[13] = new Point3D32(points[1]);
      normals[13] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[13] = new Point2D32(0.25f, 0.5f);
      points[14] = new Point3D32(points[4]);
      normals[14] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[14] = new Point2D32(0.5f, 0.75f);
      points[15] = new Point3D32(points[5]);
      normals[15] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[15] = new Point2D32(0.25f, 0.75f);

      // Front face vertices
      points[16] = new Point3D32(points[0]);
      normals[16] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[16] = new Point2D32(0.5f, 0.5f);
      points[17] = new Point3D32(points[3]);
      normals[17] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[17] = new Point2D32(0.5f, 0.25f);
      points[18] = new Point3D32(points[4]);
      normals[18] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[18] = new Point2D32(0.75f, 0.5f);
      points[19] = new Point3D32(points[7]);
      normals[19] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[19] = new Point2D32(0.75f, 0.25f);

      // Back face vertices
      points[20] = new Point3D32(points[1]);
      normals[20] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[20] = new Point2D32(0.25f, 0.5f);
      points[21] = new Point3D32(points[2]);
      normals[21] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[21] = new Point2D32(0.25f, 0.25f);
      points[22] = new Point3D32(points[5]);
      normals[22] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[22] = new Point2D32(0.0f, 0.5f);
      points[23] = new Point3D32(points[6]);
      normals[23] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[23] = new Point2D32(0.0f, 0.25f);

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

      return new TriangleMesh3DDefinition("Box Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a flat horizontal rectangle.
    * 
    * @param sizeX the rectangle size along the x-axis.
    * @param sizeY the rectangle size along the y-axis.
    * @param z     the height of the rectangle.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition FlatRectangle(double sizeX, double sizeY, double z)
   {
      return FlatRectangle((float) sizeX, (float) sizeY, (float) z);
   }

   /**
    * Creates a triangle mesh for a flat horizontal rectangle.
    * 
    * @param sizeX the rectangle size along the x-axis.
    * @param sizeY the rectangle size along the y-axis.
    * @param z     the height of the rectangle.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition FlatRectangle(float sizeX, float sizeY, float z)
   {
      return FlatRectangle(-0.5f * sizeX, -0.5f * sizeY, 0.5f * sizeX, 0.5f * sizeY, z);
   }

   /**
    * Creates a triangle mesh for a flat horizontal rectangle.
    * 
    * @param minX the rectangle's lower-bound along the x-axis.
    * @param maxX the rectangle's upper-bound along the x-axis.
    * @param minY the rectangle's lower-bound along the y-axis.
    * @param maxY the rectangle's upper-bound along the x-axis.
    * @param z    the height of the rectangle.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition FlatRectangle(double minX, double minY, double maxX, double maxY, double z)
   {
      return FlatRectangle((float) minX, (float) minY, (float) maxX, (float) maxY, (float) z);
   }

   /**
    * Creates a triangle mesh for a flat horizontal rectangle.
    * 
    * @param minX the rectangle's lower-bound along the x-axis.
    * @param maxX the rectangle's upper-bound along the x-axis.
    * @param minY the rectangle's lower-bound along the y-axis.
    * @param maxY the rectangle's upper-bound along the x-axis.
    * @param z    the height of the rectangle.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition FlatRectangle(float minX, float minY, float maxX, float maxY, float z)
   {
      Point3D32[] points = new Point3D32[4];
      Vector3D32[] normals = new Vector3D32[4];
      Point2D32[] textPoints = new Point2D32[4];

      points[0] = new Point3D32(minX, minY, z);
      points[1] = new Point3D32(maxX, minY, z);
      points[2] = new Point3D32(maxX, maxY, z);
      points[3] = new Point3D32(minX, maxY, z);

      textPoints[0] = new Point2D32(1.0f, 1.0f);
      textPoints[1] = new Point2D32(1.0f, 0.0f);
      textPoints[2] = new Point2D32(0.0f, 0.0f);
      textPoints[3] = new Point2D32(0.0f, 1.0f);

      normals[0] = new Vector3D32(0.0f, 0.0f, 1.0f);
      normals[1] = new Vector3D32(0.0f, 0.0f, 1.0f);
      normals[2] = new Vector3D32(0.0f, 0.0f, 1.0f);
      normals[3] = new Vector3D32(0.0f, 0.0f, 1.0f);

      int[] triangleIndices = new int[3 * 2];
      int index = 0;
      triangleIndices[index++] = 0;
      triangleIndices[index++] = 1;
      triangleIndices[index++] = 3;

      triangleIndices[index++] = 1;
      triangleIndices[index++] = 2;
      triangleIndices[index++] = 3;

      return new TriangleMesh3DDefinition("FlatRectangle Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D ramp.
    * <p>
    * The ramp is positioned such that its bottom face is centered at the origin.
    * </p>
    * 
    * @param description the description holding the ramp's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Ramp(Ramp3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Ramp(description.getSizeX(), description.getSizeY(), description.getSizeZ());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D ramp.
    * <p>
    * The ramp is positioned such that its bottom face is centered at the origin.
    * </p>
    * 
    * @param sizeX ramp size along the x-axis.
    * @param sizeY ramp size along the y-axis.
    * @param sizeZ ramp size along the z-axis.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Ramp(double sizeX, double sizeY, double sizeZ)
   {
      return Ramp((float) sizeX, (float) sizeY, (float) sizeZ);
   }

   /**
    * Creates a triangle mesh for a 3D ramp.
    * <p>
    * The ramp is positioned such that its bottom face is centered at the origin.
    * </p>
    * 
    * @param sizeX ramp size along the x-axis.
    * @param sizeY ramp size along the y-axis.
    * @param sizeZ ramp size along the z-axis.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Ramp(float sizeX, float sizeY, float sizeZ)
   {
      Point3D32[] points = new Point3D32[18];
      Vector3D32[] normals = new Vector3D32[18];
      Point2D32[] textPoints = new Point2D32[18];

      float tex0 = 0.0f;
      float tex1 = 1.0f / 3.0f;
      float tex2 = 2.0f / 3.0f;
      float tex3 = 1.0f;

      // Bottom face vertices
      points[0] = new Point3D32(-sizeX / 2.0f, -sizeY / 2.0f, 0.0f);
      normals[0] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[0] = new Point2D32(tex2, tex2);
      points[1] = new Point3D32(sizeX / 2.0f, -sizeY / 2.0f, 0.0f);
      normals[1] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[1] = new Point2D32(tex1, tex2);
      points[2] = new Point3D32(sizeX / 2.0f, sizeY / 2.0f, 0.0f);
      normals[2] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[2] = new Point2D32(tex1, tex1);
      points[3] = new Point3D32(-sizeX / 2.0f, sizeY / 2.0f, 0.0f);
      normals[3] = new Vector3D32(0.0f, 0.0f, -1.0f);
      textPoints[3] = new Point2D32(tex2, tex1);

      // Back face vertices
      points[4] = new Point3D32(sizeX / 2.0f, -sizeY / 2.0f, sizeZ);
      normals[4] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[4] = new Point2D32(tex0, tex2);
      points[5] = new Point3D32(sizeX / 2.0f, sizeY / 2.0f, sizeZ);
      normals[5] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[5] = new Point2D32(tex0, tex1);
      points[6] = new Point3D32(points[2]);
      normals[6] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[6] = new Point2D32(tex1, tex1);
      points[7] = new Point3D32(points[1]);
      normals[7] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[7] = new Point2D32(tex1, tex2);

      // Top face vertices
      float rampAngle = (float) Math.atan2(sizeZ, sizeX);
      points[8] = new Point3D32(points[0]);
      normals[8] = new Vector3D32(-(float) Math.sin(rampAngle), 0.0f, (float) Math.cos(rampAngle));
      textPoints[8] = new Point2D32(tex2, tex2);
      points[9] = new Point3D32(points[4]);
      normals[9] = new Vector3D32(-(float) Math.sin(rampAngle), 0.0f, (float) Math.cos(rampAngle));
      textPoints[9] = new Point2D32(tex3, tex2);
      points[10] = new Point3D32(points[5]);
      normals[10] = new Vector3D32(-(float) Math.sin(rampAngle), 0.0f, (float) Math.cos(rampAngle));
      textPoints[10] = new Point2D32(tex3, tex1);
      points[11] = new Point3D32(points[3]);
      normals[11] = new Vector3D32(-(float) Math.sin(rampAngle), 0.0f, (float) Math.cos(rampAngle));
      textPoints[11] = new Point2D32(tex2, tex1);

      // Right face vertices
      points[12] = new Point3D32(points[0]);
      normals[12] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[12] = new Point2D32(tex2, tex2);
      points[13] = new Point3D32(points[1]);
      normals[13] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[13] = new Point2D32(tex1, tex2);
      points[14] = new Point3D32(points[4]);
      normals[14] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[14] = new Point2D32(tex1, tex3);

      // Left face vertices
      points[15] = new Point3D32(points[2]);
      normals[15] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[15] = new Point2D32(tex1, tex1);
      points[16] = new Point3D32(points[3]);
      normals[16] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[16] = new Point2D32(tex2, tex1);
      points[17] = new Point3D32(points[5]);
      normals[17] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[17] = new Point2D32(tex1, tex0);

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

      return new TriangleMesh3DDefinition("Ramp Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D box which bottom and top faces are extended with a pyramid.
    * 
    * @param description the description holding the box's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition PyramidBox(PyramidBox3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = PyramidBox(description.getBoxSizeX(),
                                                           description.getBoxSizeY(),
                                                           description.getBoxSizeZ(),
                                                           description.getPyramidHeight());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D box which bottom and top faces are extended with a pyramid.
    * 
    * @param boxSizeX      box size along the x-axis.
    * @param boxSizeY      box size along the y-axis.
    * @param boxSizeZ      box size along the z-axis.
    * @param pyramidHeight the height for each pyramid.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition PyramidBox(double boxSizeX, double boxSizeY, double boxSizeZ, double pyramidHeight)
   {
      return PyramidBox((float) boxSizeX, (float) boxSizeY, (float) boxSizeZ, (float) pyramidHeight);
   }

   /**
    * Creates a triangle mesh for a 3D box which bottom and top faces are extended with a pyramid.
    * 
    * @param boxSizeX      box size along the x-axis.
    * @param boxSizeY      box size along the y-axis.
    * @param boxSizeZ      box size along the z-axis.
    * @param pyramidHeight the height for each pyramid.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition PyramidBox(float boxSizeX, float boxSizeY, float boxSizeZ, float pyramidHeight)
   {
      Point3D32 points[] = new Point3D32[40];
      Vector3D32[] normals = new Vector3D32[40];
      Point2D32 textPoints[] = new Point2D32[40];
      float totalHeight = 2.0f * pyramidHeight + boxSizeZ;

      // Box front face
      points[0] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[0] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[0] = new Point2D32(0.75f, 1.0f - pyramidHeight / totalHeight);
      points[1] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[1] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[1] = new Point2D32(0.75f, pyramidHeight / totalHeight);
      points[2] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[2] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[2] = new Point2D32(0.5f, pyramidHeight / totalHeight);
      points[3] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[3] = new Vector3D32(-1.0f, 0.0f, 0.0f);
      textPoints[3] = new Point2D32(0.5f, 1.0f - pyramidHeight / totalHeight);

      // Box back face
      points[4] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[4] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[4] = new Point2D32(0.0f, 1.0f - pyramidHeight / totalHeight);
      points[5] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[5] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[5] = new Point2D32(0.0f, pyramidHeight / totalHeight);
      points[6] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[6] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[6] = new Point2D32(0.25f, pyramidHeight / totalHeight);
      points[7] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[7] = new Vector3D32(1.0f, 0.0f, 0.0f);
      textPoints[7] = new Point2D32(0.25f, 1.0f - pyramidHeight / totalHeight);

      // Box left face
      points[8] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[8] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[8] = new Point2D32(0.5f, 1.0f - pyramidHeight / totalHeight);
      points[9] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[9] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[9] = new Point2D32(0.5f, pyramidHeight / totalHeight);
      points[10] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[10] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[10] = new Point2D32(0.25f, pyramidHeight / totalHeight);
      points[11] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[11] = new Vector3D32(0.0f, 1.0f, 0.0f);
      textPoints[11] = new Point2D32(0.25f, 1.0f - pyramidHeight / totalHeight);

      // Box right face
      points[12] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[12] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[12] = new Point2D32(0.75f, 1.0f - pyramidHeight / totalHeight);
      points[13] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[13] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[13] = new Point2D32(0.75f, pyramidHeight / totalHeight);
      points[14] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[14] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[14] = new Point2D32(1.0f, pyramidHeight / totalHeight);
      points[15] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[15] = new Vector3D32(0.0f, -1.0f, 0.0f);
      textPoints[15] = new Point2D32(1.0f, 1.0f - pyramidHeight / totalHeight);

      float frontBackAngle = (float) Math.atan2(boxSizeX / 2.0, pyramidHeight);
      float leftRightAngle = (float) Math.atan2(boxSizeY / 2.0, pyramidHeight);

      // Top pyramid
      // Front face
      points[16] = new Point3D32(0.0f, 0.0f, 0.5f * boxSizeZ + pyramidHeight);
      normals[16] = new Vector3D32(-(float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[16] = new Point2D32(0.625f, 0.0f);
      points[17] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[17] = new Vector3D32(-(float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[17] = new Point2D32(0.75f, pyramidHeight / totalHeight);
      points[18] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[18] = new Vector3D32(-(float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[18] = new Point2D32(0.5f, pyramidHeight / totalHeight);

      // Back face
      points[19] = new Point3D32(0.0f, 0.0f, 0.5f * boxSizeZ + pyramidHeight);
      normals[19] = new Vector3D32((float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[19] = new Point2D32(0.125f, 0.0f);
      points[20] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[20] = new Vector3D32((float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[20] = new Point2D32(0.0f, pyramidHeight / totalHeight);
      points[21] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[21] = new Vector3D32((float) Math.cos(frontBackAngle), 0.0f, (float) Math.sin(frontBackAngle));
      textPoints[21] = new Point2D32(0.25f, pyramidHeight / totalHeight);

      // Left face
      points[22] = new Point3D32(0.0f, 0.0f, 0.5f * boxSizeZ + pyramidHeight);
      normals[22] = new Vector3D32(0.0f, (float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[22] = new Point2D32(0.375f, 0.0f);
      points[23] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[23] = new Vector3D32(0.0f, (float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[23] = new Point2D32(0.5f, pyramidHeight / totalHeight);
      points[24] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[24] = new Vector3D32(0.0f, (float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[24] = new Point2D32(0.25f, pyramidHeight / totalHeight);

      // Right face
      points[25] = new Point3D32(0.0f, 0.0f, 0.5f * boxSizeZ + pyramidHeight);
      normals[25] = new Vector3D32(0.0f, -(float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[25] = new Point2D32(0.875f, 0.0f);
      points[26] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[26] = new Vector3D32(0.0f, -(float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[26] = new Point2D32(0.75f, pyramidHeight / totalHeight);
      points[27] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, 0.5f * boxSizeZ);
      normals[27] = new Vector3D32(0.0f, -(float) Math.cos(leftRightAngle), (float) Math.sin(leftRightAngle));
      textPoints[27] = new Point2D32(1.0f, pyramidHeight / totalHeight);

      // Bottom pyramid
      // Front face
      points[28] = new Point3D32(0.0f, 0.0f, -pyramidHeight - 0.5f * boxSizeZ);
      normals[28] = new Vector3D32(-(float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[28] = new Point2D32(0.625f, 1.0f);
      points[29] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[29] = new Vector3D32(-(float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[29] = new Point2D32(0.75f, 1.0f - pyramidHeight / totalHeight);
      points[30] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[30] = new Vector3D32(-(float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[30] = new Point2D32(0.5f, 1.0f - pyramidHeight / totalHeight);

      // Back face
      points[31] = new Point3D32(0.0f, 0.0f, -pyramidHeight - 0.5f * boxSizeZ);
      normals[31] = new Vector3D32((float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[31] = new Point2D32(0.125f, 1.0f);
      points[32] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[32] = new Vector3D32((float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[32] = new Point2D32(0.0f, 1.0f - pyramidHeight / totalHeight);
      points[33] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[33] = new Vector3D32((float) Math.cos(frontBackAngle), 0.0f, -(float) Math.sin(frontBackAngle));
      textPoints[33] = new Point2D32(0.25f, 1.0f - pyramidHeight / totalHeight);

      // Left face
      points[34] = new Point3D32(0.0f, 0.0f, -pyramidHeight - 0.5f * boxSizeZ);
      normals[34] = new Vector3D32(0.0f, (float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[34] = new Point2D32(0.375f, 1.0f);
      points[35] = new Point3D32(-boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[35] = new Vector3D32(0.0f, (float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[35] = new Point2D32(0.5f, 1.0f - pyramidHeight / totalHeight);
      points[36] = new Point3D32(boxSizeX / 2.0f, boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[36] = new Vector3D32(0.0f, (float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[36] = new Point2D32(0.25f, 1.0f - pyramidHeight / totalHeight);

      // Right face
      points[37] = new Point3D32(0.0f, 0.0f, -pyramidHeight - 0.5f * boxSizeZ);
      normals[37] = new Vector3D32(0.0f, -(float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[37] = new Point2D32(0.875f, 1.0f);
      points[38] = new Point3D32(-boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[38] = new Vector3D32(0.0f, -(float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[38] = new Point2D32(0.75f, 1.0f - pyramidHeight / totalHeight);
      points[39] = new Point3D32(boxSizeX / 2.0f, -boxSizeY / 2.0f, -0.5f * boxSizeZ);
      normals[39] = new Vector3D32(0.0f, -(float) Math.cos(leftRightAngle), -(float) Math.sin(leftRightAngle));
      textPoints[39] = new Point2D32(1.0f, 1.0f - pyramidHeight / totalHeight);

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

      return new TriangleMesh3DDefinition("PyramidBox Factory", points, textPoints, normals, polygonIndices);
   }

   /**
    * Create a triangle mesh for a 3D line segment.
    * <p>
    * The line segment is implemented as an elongated 3D box.
    * </p>
    * 
    * @param lineSegment used to define the mesh end points.
    * @param width       the thickness of the line.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Line(LineSegment3DReadOnly lineSegment, double width)
   {
      return Line(lineSegment.getFirstEndpoint(), lineSegment.getSecondEndpoint(), width);
   }

   /**
    * Create a triangle mesh for a 3D line segment.
    * <p>
    * The line segment is implemented as an elongated 3D box.
    * </p>
    *
    * @param point0 the first endpoint of the line segment.
    * @param point1 the second endpoint of the line segment.
    * @param width  the thickness of the line.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Line(Point3DReadOnly point0, Point3DReadOnly point1, double width)
   {
      return Line(point0.getX(), point0.getY(), point0.getZ(), point1.getX(), point1.getY(), point1.getZ(), width);
   }

   /**
    * Create a triangle mesh for a 3D line segment.
    * <p>
    * The line segment is implemented as an elongated 3D box.
    * </p>
    * 
    * @param x0    the x-coordinate of the first endpoint for the line segment.
    * @param y0    the y-coordinate of the first endpoint for the line segment.
    * @param z0    the z-coordinate of the first endpoint for the line segment.
    * @param x1    the x-coordinate of the second endpoint for the line segment.
    * @param y1    the y-coordinate of the second endpoint for the line segment.
    * @param z1    the z-coordinate of the second endpoint for the line segment.
    * @param width the thickness of the line.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Line(double x0, double y0, double z0, double x1, double y1, double z1, double width)
   {
      return Line((float) x0, (float) y0, (float) z0, (float) x1, (float) y1, (float) z1, (float) width);
   }

   /**
    * Create a triangle mesh for a 3D line segment.
    * <p>
    * The line segment is implemented as an elongated 3D box.
    * </p>
    * 
    * @param x0    the x-coordinate of the first endpoint for the line segment.
    * @param y0    the y-coordinate of the first endpoint for the line segment.
    * @param z0    the z-coordinate of the first endpoint for the line segment.
    * @param x1    the x-coordinate of the second endpoint for the line segment.
    * @param y1    the y-coordinate of the second endpoint for the line segment.
    * @param z1    the z-coordinate of the second endpoint for the line segment.
    * @param width the thickness of the line.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Line(float x0, float y0, float z0, float x1, float y1, float z1, float width)
   {
      Vector3D32 lineDirection = new Vector3D32(x1 - x0, y1 - y0, z1 - z0);
      float lineLength = (float) lineDirection.length();
      lineDirection.scale(1.0f / lineLength);
      TriangleMesh3DDefinition line = Box(width, width, lineLength, false);
      line.setName("Line Factory");
      Point3D32[] vertices = line.getVertices();
      Vector3D32[] normals = line.getNormals();

      float yaw;
      float pitch;
      if (Math.abs(lineDirection.getZ()) < 1.0 - 1.0e-7)
      {
         yaw = (float) Math.atan2(lineDirection.getY(), lineDirection.getX());
         double xyLength = Math.sqrt(lineDirection.getX() * lineDirection.getX() + lineDirection.getY() * lineDirection.getY());
         pitch = (float) Math.atan2(xyLength, lineDirection.getZ());
      }
      else
      {
         yaw = 0.0f;
         pitch = lineDirection.getZ() >= 0.0 ? 0.0f : (float) Math.PI;
      }

      float cYaw = (float) Math.cos(yaw);
      float sYaw = (float) Math.sin(yaw);

      float cPitch = (float) Math.cos(pitch);
      float sPitch = (float) Math.sin(pitch);

      float rxx = cYaw * cPitch;
      float rxy = -sYaw;
      float rxz = cYaw * sPitch;
      float ryx = sYaw * cPitch;
      float ryy = cYaw;
      float ryz = sYaw * sPitch;
      float rzx = -sPitch;
      float rzz = cPitch;

      for (int i = 0; i < vertices.length; i++)
      {
         Point3D32 vertex = vertices[i];
         float vx = vertex.getX32();
         float vy = vertex.getY32();
         float vz = vertex.getZ32();
         vertex.setX(x0 + rxx * vx + rxy * vy + rxz * vz);
         vertex.setY(y0 + ryx * vx + ryy * vy + ryz * vz);
         vertex.setZ(z0 + rzx * vx + rzz * vz);
      }

      for (int i = 0; i < normals.length; i++)
      {
         Vector3D32 normal = normals[i];
         float vx = normal.getX32();
         float vy = normal.getY32();
         float vz = normal.getZ32();
         normal.setX(rxx * vx + rxy * vy + rxz * vz);
         normal.setY(ryx * vx + ryy * vy + ryz * vz);
         normal.setZ(rzx * vx + rzz * vz);
      }
      return line;
   }

   /**
    * Creates a triangle mesh for a 3D capsule with its ends being half ellipsoids.
    * <p>
    * The capsule's axis is aligned with the z-axis and it is centered at the origin.
    * </p>
    *
    * @param description the description holding the capsule's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Capsule(Capsule3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Capsule(description.getLength(),
                                                        description.getRadiusX(),
                                                        description.getRadiusY(),
                                                        description.getRadiusZ(),
                                                        description.getResolution(),
                                                        description.getResolution());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D capsule with its ends being half ellipsoids.
    * <p>
    * The capsule's axis is aligned with the z-axis and it is centered at the origin.
    * </p>
    *
    * @param height              the capsule's height or length. Distance separating the center of the
    *                            two half ellipsoids.
    * @param radiusX             radius of the capsule along the x-axis.
    * @param radiusY             radius of the capsule along the y-axis.
    * @param radiusZ             radius of the capsule along the z-axis.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Capsule(double height, double radiusX, double radiusY, double radiusZ, int latitudeResolution,
                                                  int longitudeResolution)
   {
      return Capsule((float) height, (float) radiusX, (float) radiusY, (float) radiusZ, latitudeResolution, longitudeResolution);
   }

   /**
    * Creates a triangle mesh for a 3D capsule with its ends being half ellipsoids.
    * <p>
    * The capsule's axis is aligned with the z-axis and it is centered at the origin.
    * </p>
    *
    * @param height              the capsule's height or length. Distance separating the center of the
    *                            two half ellipsoids.
    * @param radiusX             radius of the capsule along the x-axis.
    * @param radiusY             radius of the capsule along the y-axis.
    * @param radiusZ             radius of the capsule along the z-axis.
    * @param latitudeResolution  the resolution along the vertical axis, i.e. z-axis.
    * @param longitudeResolution the resolution around the vertical axis, i.e. the number of vertices
    *                            per latitude.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Capsule(float height, float radiusX, float radiusY, float radiusZ, int latitudeResolution, int longitudeResolution)
   {
      if (latitudeResolution % 2 != 0)
         latitudeResolution++;
      if (longitudeResolution % 2 != 1)
         longitudeResolution++;

      // Reminder of longitude and latitude: http://www.geographyalltheway.com/ks3_geography/maps_atlases/longitude_latitude.htm
      int numberOfVertices = latitudeResolution * longitudeResolution;
      Point3D32 points[] = new Point3D32[numberOfVertices];
      Vector3D32[] normals = new Vector3D32[numberOfVertices];
      Point2D32 textPoints[] = new Point2D32[numberOfVertices];

      float texRatio = radiusZ / (2.0f * radiusZ + height);

      float halfHeight = 0.5f * height;

      for (int longitudeIndex = 0; longitudeIndex < longitudeResolution; longitudeIndex++)
      {
         float longitudeAngle = TwoPi * ((float) longitudeIndex / (float) (longitudeResolution - 1.0f));
         float textureX = (float) longitudeIndex / (float) (longitudeResolution - 1);

         // Bottom hemi-ellipsoid
         for (int latitudeIndex = 1; latitudeIndex < latitudeResolution / 2; latitudeIndex++)
         {
            float latitudeAngle = (float) (-HalfPi + Math.PI * ((float) latitudeIndex / (latitudeResolution - 1.0f)));

            float cosLongitude = (float) Math.cos(longitudeAngle);
            float sinLongitude = (float) Math.sin(longitudeAngle);
            float cosLatitude = (float) Math.cos(latitudeAngle);
            float sinLatitude = (float) Math.sin(latitudeAngle);

            int currentIndex = latitudeIndex * longitudeResolution + longitudeIndex;
            float normalX = cosLongitude * cosLatitude;
            float normalY = sinLongitude * cosLatitude;
            float normalZ = sinLatitude;
            float vertexX = radiusX * normalX;
            float vertexY = radiusY * normalY;
            float vertexZ = radiusZ * normalZ - halfHeight;
            points[currentIndex] = new Point3D32(vertexX, vertexY, vertexZ);
            normals[currentIndex] = new Vector3D32(normalX, normalY, normalZ);

            float textureY = 1.0f - (1.0f + sinLatitude) * texRatio;
            textPoints[currentIndex] = new Point2D32(textureX, textureY);
         }

         // Top hemi-ellipsoid
         for (int latitudeIndex = 0; latitudeIndex < latitudeResolution / 2 - 1; latitudeIndex++)
         {
            float latitudeAngle = (float) (Math.PI * ((float) latitudeIndex / (latitudeResolution - 1.0f)));

            float cosLongitude = (float) Math.cos(longitudeAngle);
            float sinLongitude = (float) Math.sin(longitudeAngle);
            float cosLatitude = (float) Math.cos(latitudeAngle);
            float sinLatitude = (float) Math.sin(latitudeAngle);

            int currentIndex = (latitudeResolution / 2 + latitudeIndex) * longitudeResolution + longitudeIndex;
            float normalX = cosLongitude * cosLatitude;
            float normalY = sinLongitude * cosLatitude;
            float normalZ = sinLatitude;
            float vertexX = radiusX * normalX;
            float vertexY = radiusY * normalY;
            float vertexZ = radiusZ * normalZ + halfHeight;
            points[currentIndex] = new Point3D32(vertexX, vertexY, vertexZ);
            normals[currentIndex] = new Vector3D32(normalX, normalY, normalZ);

            float textureY = (1.0f - sinLatitude) * texRatio;
            textPoints[currentIndex] = new Point2D32(textureX, textureY);
         }

         textureX += 0.5f / (longitudeResolution - 1.0f);
         // South pole
         int southPoleIndex = longitudeIndex;
         points[southPoleIndex] = new Point3D32(0.0f, 0.0f, -radiusZ - halfHeight);
         normals[southPoleIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
         textPoints[southPoleIndex] = new Point2D32(textureX, 1.0f - 1.0f / 256.0f);

         // North pole
         int northPoleIndex = (latitudeResolution - 1) * longitudeResolution + longitudeIndex;
         points[northPoleIndex] = new Point3D32(0.0f, 0.0f, radiusZ + halfHeight);
         normals[northPoleIndex] = new Vector3D32(0.0f, 0.0f, 1.0f);
         textPoints[northPoleIndex] = new Point2D32(textureX, 1.0f / 256.0f);
      }

      int numberOfTriangles = 2 * latitudeResolution * longitudeResolution + 1 * longitudeResolution;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Top hemi-ellipsoid Mid-latitude faces
      for (int latitudeIndex = 1; latitudeIndex < latitudeResolution - 1; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < longitudeResolution; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % longitudeResolution;
            int nextLatitudeIndex = latitudeIndex + 1;

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * longitudeResolution + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * longitudeResolution + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeResolution + longitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * longitudeResolution + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeResolution + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeResolution + longitudeIndex;
         }
      }

      // South pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeResolution - 1; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeResolution;
         triangleIndices[index++] = longitudeIndex;
         triangleIndices[index++] = longitudeResolution + nextLongitudeIndex;
         triangleIndices[index++] = longitudeResolution + longitudeIndex;
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeResolution - 1; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeResolution;
         triangleIndices[index++] = (latitudeResolution - 1) * longitudeResolution + longitudeIndex;
         triangleIndices[index++] = (latitudeResolution - 2) * longitudeResolution + longitudeIndex;
         triangleIndices[index++] = (latitudeResolution - 2) * longitudeResolution + nextLongitudeIndex;
      }

      return new TriangleMesh3DDefinition("Capsule Factory", points, textPoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D regular tetrahedron.
    * <p>
    * Its base is centered at the origin.
    * </p>
    * 
    * @param description the description holding the tetrahedron's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Tetrahedron(Tetrahedron3DDefinition description)
   {
      TriangleMesh3DDefinition meshDataHolder = Tetrahedron(description.getEdgeLength());
      if (meshDataHolder != null)
         meshDataHolder.setName(description.getName());
      return meshDataHolder;
   }

   /**
    * Creates a triangle mesh for a 3D regular tetrahedron.
    * <p>
    * The tetrahedron is centered at the origin.
    * </p>
    * 
    * @param edgeLength length of the tetrahedron's edges.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Tetrahedron(double edgeLength)
   {
      return Tetrahedron((float) edgeLength);
   }

   private static final float TETRAHEDRON_FACE_EDGE_FACE_ANGLE = (float) Math.acos(ONE_THIRD);
   private static final float TETRAHEDRON_SINE_FACE_EDGE_FACE_ANGLE = (float) Math.sin(TETRAHEDRON_FACE_EDGE_FACE_ANGLE);

   /**
    * Creates a triangle mesh for a 3D regular tetrahedron.
    * <p>
    * The tetrahedron is centered at the origin.
    * </p>
    * 
    * @param edgeLength length of the tetrahedron's edges.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition Tetrahedron(float edgeLength)
   {
      /*
       * @formatter:off
       * Base vertices ordering
       *     0
       *    / \
       *   /   \
       *  /     \
       * 2 ----- 1
       * @formatter:on
       */
      float height = THIRD_SQRT6 * edgeLength;
      float topHeight = FOURTH_SQRT6 * edgeLength;
      float baseHeight = topHeight - height;

      float halfEdgeLength = 0.5f * edgeLength;

      float cosFaceEdgeFace = ONE_THIRD;
      float sinFaceEdgeFace = TETRAHEDRON_SINE_FACE_EDGE_FACE_ANGLE;

      float cosEdgeVertexEdge = 0.5f;
      float sinEdgeVertexEdge = HALF_SQRT3;

      Point3D32 topVertex = new Point3D32(0.0f, 0.0f, topHeight);
      Point3D32 baseVertex0 = new Point3D32(edgeLength * THIRD_SQRT3, 0.0f, baseHeight);
      Point3D32 baseVertex1 = new Point3D32(-edgeLength * SIXTH_SQRT3, halfEdgeLength, baseHeight);
      Point3D32 baseVertex2 = new Point3D32(-edgeLength * SIXTH_SQRT3, -halfEdgeLength, baseHeight);

      Vector3D32 frontNormal = new Vector3D32(-sinFaceEdgeFace, 0.0f, cosFaceEdgeFace);
      Vector3D32 rightNormal = new Vector3D32(sinFaceEdgeFace * sinEdgeVertexEdge, sinFaceEdgeFace * cosEdgeVertexEdge, cosFaceEdgeFace);
      Vector3D32 leftNormal = new Vector3D32(sinFaceEdgeFace * sinEdgeVertexEdge, -sinFaceEdgeFace * cosEdgeVertexEdge, cosFaceEdgeFace);
      Vector3D32 baseNormal = new Vector3D32(0.0f, 0.0f, -1.0f);

      int numberOfVertices = 12;
      Point3D32[] vertices = new Point3D32[numberOfVertices];
      Vector3D32[] normals = new Vector3D32[numberOfVertices];
      Point2D32[] texturePoints = new Point2D32[numberOfVertices];

      // Front face
      vertices[0] = new Point3D32(baseVertex2);
      normals[0] = new Vector3D32(frontNormal);
      texturePoints[0] = new Point2D32(0.25f, 0.5f);
      vertices[1] = new Point3D32(baseVertex1);
      normals[1] = new Vector3D32(frontNormal);
      texturePoints[1] = new Point2D32(0.75f, 0.5f);
      vertices[2] = new Point3D32(topVertex);
      normals[2] = new Vector3D32(frontNormal);
      texturePoints[2] = new Point2D32(0.5f, 1.0f);

      // Right face
      vertices[3] = new Point3D32(baseVertex1);
      normals[3] = new Vector3D32(rightNormal);
      texturePoints[3] = new Point2D32(0.75f, 0.5f);
      vertices[4] = new Point3D32(baseVertex0);
      normals[4] = new Vector3D32(rightNormal);
      texturePoints[4] = new Point2D32(0.5f, 0.0f);
      vertices[5] = new Point3D32(topVertex);
      normals[5] = new Vector3D32(rightNormal);
      texturePoints[5] = new Point2D32(1.0f, 0.0f);

      // Left face
      vertices[6] = new Point3D32(baseVertex0);
      normals[6] = new Vector3D32(leftNormal);
      texturePoints[6] = new Point2D32(0.5f, 0.0f);
      vertices[7] = new Point3D32(baseVertex2);
      normals[7] = new Vector3D32(leftNormal);
      texturePoints[7] = new Point2D32(0.25f, 0.5f);
      vertices[8] = new Point3D32(topVertex);
      normals[8] = new Vector3D32(leftNormal);
      texturePoints[8] = new Point2D32(0.0f, 0.0f);

      // Bottom face
      vertices[9] = new Point3D32(baseVertex0);
      normals[9] = new Vector3D32(baseNormal);
      texturePoints[9] = new Point2D32(0.5f, 0.0f);
      vertices[10] = new Point3D32(baseVertex1);
      normals[10] = new Vector3D32(baseNormal);
      texturePoints[10] = new Point2D32(0.75f, 0.5f);
      vertices[11] = new Point3D32(baseVertex2);
      normals[11] = new Vector3D32(baseNormal);
      texturePoints[11] = new Point2D32(0.25f, 0.5f);

      int numberOfTriangles = 4;

      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;
      // Front face
      triangleIndices[index++] = 0;
      triangleIndices[index++] = 2;
      triangleIndices[index++] = 1;
      // Right face
      triangleIndices[index++] = 3;
      triangleIndices[index++] = 5;
      triangleIndices[index++] = 4;
      // Left face
      triangleIndices[index++] = 6;
      triangleIndices[index++] = 8;
      triangleIndices[index++] = 7;
      // Bottom face
      triangleIndices[index++] = 9;
      triangleIndices[index++] = 11;
      triangleIndices[index++] = 10;

      return new TriangleMesh3DDefinition("Tetrahedron Factory", vertices, texturePoints, normals, triangleIndices);
   }

   /**
    * Creates a triangle mesh for a 3D convex polytope.
    * <p>
    * The texture mapping is computed from the spherical coordinates of the vertices and may not be
    * appropriate depending on the polytope.
    * </p>
    * 
    * @param description the description holding the polytope's properties.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition ConvexPolytope(ConvexPolytope3DDefinition description)
   {
      return ConvexPolytope(description.getConvexPolytope());
   }

   /**
    * Creates a triangle mesh for a 3D convex polytope.
    * <p>
    * The texture mapping is computed from the spherical coordinates of the vertices and may not be
    * appropriate depending on the polytope.
    * </p>
    * 
    * @param convexPolytope the polytope to create the triangle mesh for. Not modified.
    * @return the generic triangle mesh.
    */
   public static TriangleMesh3DDefinition ConvexPolytope(ConvexPolytope3DReadOnly convexPolytope)
   {
      if (convexPolytope == null)
         return null;

      int numberOfVertices = convexPolytope.getFaces().stream().mapToInt(Face3DReadOnly::getNumberOfEdges).sum();
      Point3D32[] vertices = new Point3D32[numberOfVertices];
      Vector3D32[] normals = new Vector3D32[numberOfVertices];
      Point2D32[] texturePoints = new Point2D32[numberOfVertices];

      int numberOfTriangles = numberOfVertices - 2 * convexPolytope.getNumberOfFaces();
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int vertexOffset = 0;
      int triangleOffset = 0;
      Vector3D direction = new Vector3D();

      for (int faceIndex = 0; faceIndex < convexPolytope.getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = convexPolytope.getFace(faceIndex);

         double minLongitude = Double.POSITIVE_INFINITY;
         double[] longitudes = new double[face.getNumberOfEdges()];

         for (int vertexIndex = 0; vertexIndex < face.getNumberOfEdges(); vertexIndex++)
         {
            Vertex3DReadOnly vertex = face.getVertex(vertexIndex);
            vertices[vertexOffset + vertexIndex] = new Point3D32(vertex);
            normals[vertexOffset + vertexIndex] = new Vector3D32(face.getNormal());

            direction.sub(vertex, convexPolytope.getCentroid());
            direction.normalize();

            double longitude = Math.atan2(direction.getY(), direction.getX());
            longitudes[vertexIndex] = longitude;
            minLongitude = Math.min(longitude, minLongitude);
            float textureY = 0.5f * (1.0f - direction.getZ32());
            texturePoints[vertexOffset + vertexIndex] = new Point2D32(0.0f, textureY);
         }

         for (int vertexIndex = 0; vertexIndex < face.getNumberOfEdges(); vertexIndex++)
         {
            double longitude = minLongitude + EuclidCoreTools.angleDifferenceMinusPiToPi(longitudes[vertexIndex], minLongitude);
            float textureX = (float) (0.5 * (longitude / Math.PI + 1.0));
            texturePoints[vertexOffset + vertexIndex].setX(textureX);
         }

         for (int i = 2; i < face.getNumberOfEdges(); i++)
         {
            triangleIndices[triangleOffset++] = vertexOffset;
            triangleIndices[triangleOffset++] = vertexOffset + i;
            triangleIndices[triangleOffset++] = vertexOffset + i - 1;
         }

         vertexOffset += face.getNumberOfEdges();
      }

      return new TriangleMesh3DDefinition("ConvexPolytope Factory", vertices, texturePoints, normals, triangleIndices);
   }

   /*
    * TODO: The following is for drawing STP shapes. Needs some cleanup.
    */

   public static TriangleMesh3DDefinition toSTPBox3DMesh(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ, double smallRadius,
                                                         double largeRadius, boolean highlightLimits)
   {
      return combine(true, false, toSTPBox3DMeshes(pose, sizeX, sizeY, sizeZ, smallRadius, largeRadius, highlightLimits));
   }

   public static TriangleMesh3DDefinition[] toSTPBox3DMeshes(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ, double smallRadius,
                                                             double largeRadius, boolean highlightLimits)
   {
      STPBox3D stpBox3D = new STPBox3D(sizeX, sizeY, sizeZ);
      if (pose != null)
         stpBox3D.getPose().set(pose);
      BoxPolytope3DView boxPolytope = stpBox3D.asConvexPolytope();
      return toSTPConvexPolytope3DMeshes(boxPolytope, smallRadius, largeRadius, highlightLimits);
   }

   public static TriangleMesh3DDefinition toSTPCapsule3DMesh(RigidBodyTransformReadOnly pose, double radius, double length, double smallRadius,
                                                             double largeRadius, boolean highlightLimits)
   {
      return combine(true, false, toSTPCapsule3DMeshes(pose, radius, length, smallRadius, largeRadius, highlightLimits));
   }

   public static TriangleMesh3DDefinition[] toSTPCapsule3DMeshes(RigidBodyTransformReadOnly pose, double radius, double length, double smallRadius,
                                                                 double largeRadius, boolean highlightLimits)
   {
      List<TriangleMesh3DDefinition> faceMeshes = new ArrayList<>();

      UnitVector3D axis = new UnitVector3D(Axis3D.Z);
      Point3D position = new Point3D();
      if (pose != null)
      {
         pose.transform(axis);
         position.set(pose.getTranslation());
      }
      Point3D topCenter = new Point3D();
      topCenter.scaleAdd(0.5 * length, axis, position);
      Point3D bottomCenter = new Point3D();
      bottomCenter.scaleAdd(-0.5 * length, axis, position);

      // Side face
      Vector3D axisOrthogonal = newOrthogonalVector(axis);
      double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, length);
      Point3D sphereCenter = new Point3D();
      sphereCenter.scaleAdd(-sphereOffset, axisOrthogonal, position);

      UnitVector3D startDirection = new UnitVector3D();
      UnitVector3D endDirection = new UnitVector3D();

      startDirection.sub(bottomCenter, sphereCenter);
      endDirection.sub(topCenter, sphereCenter);

      TriangleMesh3DDefinition arc = toArcPointsAndNormals(sphereCenter, largeRadius, startDirection, endDirection, 64);
      faceMeshes.add(applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false));

      if (highlightLimits)
      {
         double limitPositionOnAxis = 0.5 * length * largeRadius / (largeRadius - smallRadius);
         double limitRadius = sphereOffset * smallRadius / (largeRadius - smallRadius);
         TriangleMesh3DDefinition sideLimitMesh = ArcTorus(0.0, TwoPi, limitRadius, 0.001, 64);
         sideLimitMesh = rotate(sideLimitMesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
         sideLimitMesh = translate(sideLimitMesh, position);
         Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(limitPositionOnAxis, axis, v));
         faceMeshes.add(sideLimitMesh.copy());

         Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(-2.0 * limitPositionOnAxis, axis, v));
         faceMeshes.add(sideLimitMesh);
      }

      // Cap faces
      startDirection.set(axis);

      arc = toArcPointsAndNormals(topCenter, smallRadius, endDirection, startDirection, 64);
      TriangleMesh3DDefinition capMesh = applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false);
      faceMeshes.add(capMesh.copy());

      // Flipping the meshes around to draw the bottom cap
      RotationMatrix flipRotation = new RotationMatrix();
      flipRotation.setAxisAngle(axisOrthogonal.getX(), axisOrthogonal.getY(), axisOrthogonal.getZ(), Math.PI);
      Arrays.asList(capMesh.getVertices()).forEach(v ->
      {
         v.sub(position);
         flipRotation.transform(v);
         v.add(position);
      });
      Arrays.asList(capMesh.getNormals()).forEach(n -> flipRotation.transform(n));
      faceMeshes.add(capMesh);

      return new TriangleMesh3DDefinition[] {combine(true, false, faceMeshes)};
   }

   public static TriangleMesh3DDefinition toSTPCylinder3DMesh(RigidBodyTransformReadOnly pose, double radius, double length, double smallRadius,
                                                              double largeRadius, boolean highlightLimits)
   {
      return combine(true, false, toSTPCylinder3DMeshes(pose, radius, length, smallRadius, largeRadius, highlightLimits));
   }

   public static TriangleMesh3DDefinition[] toSTPCylinder3DMeshes(RigidBodyTransformReadOnly pose, double radius, double length, double smallRadius,
                                                                  double largeRadius, boolean highlightLimits)
   {
      List<TriangleMesh3DDefinition> faceMeshes = new ArrayList<>();
      List<TriangleMesh3DDefinition> edgeMeshes = new ArrayList<>();

      UnitVector3D axis = new UnitVector3D(Axis3D.Z);
      Point3D position = new Point3D();
      if (pose != null)
      {
         axis.applyTransform(pose);
         position.set(pose.getTranslation());
      }
      Point3D topCenter = new Point3D();
      topCenter.scaleAdd(0.5 * length, axis, position);
      Point3D bottomCenter = new Point3D();
      bottomCenter.scaleAdd(-0.5 * length, axis, position);

      { // Side face
         Vector3D axisOrthogonal = newOrthogonalVector(axis);
         double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, length);
         Point3D sphereCenter = new Point3D();
         sphereCenter.scaleAdd(-sphereOffset + radius, axisOrthogonal, position);

         UnitVector3D startDirection = new UnitVector3D();
         UnitVector3D endDirection = new UnitVector3D();

         startDirection.scaleAdd(radius, axisOrthogonal, bottomCenter);
         startDirection.sub(sphereCenter);
         endDirection.scaleAdd(radius, axisOrthogonal, topCenter);
         endDirection.sub(sphereCenter);

         TriangleMesh3DDefinition arc = toArcPointsAndNormals(sphereCenter, largeRadius, startDirection, endDirection, 64);
         faceMeshes.add(applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false));

         if (highlightLimits)
         {
            double limitPositionOnAxis = 0.5 * length * largeRadius / (largeRadius - smallRadius);
            double limitRadius = radius + sphereOffset * smallRadius / (largeRadius - smallRadius);
            TriangleMesh3DDefinition sideLimitMesh = ArcTorus(0.0, TwoPi, limitRadius, 0.001, 64);
            sideLimitMesh = rotate(sideLimitMesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
            sideLimitMesh = translate(sideLimitMesh, position);
            Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(limitPositionOnAxis, axis, v));
            faceMeshes.add(sideLimitMesh.copy());

            Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(-2.0 * limitPositionOnAxis, axis, v));
            faceMeshes.add(sideLimitMesh);
         }
      }

      { // Cap faces
         double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * radius);
         Point3D sphereCenter = new Point3D();
         sphereCenter.scaleAdd(-sphereOffset, axis, topCenter);

         Vector3D axisOrthogonal = newOrthogonalVector(axis);

         UnitVector3D boundaryDirection = new UnitVector3D();
         boundaryDirection.scaleAdd(radius, axisOrthogonal, topCenter);
         boundaryDirection.sub(sphereCenter);

         TriangleMesh3DDefinition arc = toArcPointsAndNormals(sphereCenter, largeRadius, boundaryDirection, axis, 64);
         TriangleMesh3DDefinition capMesh = applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false);
         faceMeshes.add(capMesh.copy());

         // Flipping the meshes around to draw the bottom cap
         RotationMatrix flipRotation = new RotationMatrix();
         flipRotation.setAxisAngle(axisOrthogonal.getX(), axisOrthogonal.getY(), axisOrthogonal.getZ(), Math.PI);
         Arrays.asList(capMesh.getVertices()).forEach(v ->
         {
            v.sub(position);
            flipRotation.transform(v);
            v.add(position);
         });
         Arrays.asList(capMesh.getNormals()).forEach(n -> flipRotation.transform(n));
         faceMeshes.add(capMesh);

         if (highlightLimits)
         {
            double limitPositionOnAxis = 0.5 * length + sphereOffset * smallRadius / (largeRadius - smallRadius);
            double limitRadius = radius * largeRadius / (largeRadius - smallRadius);
            TriangleMesh3DDefinition capLimitMesh = ArcTorus(0.0, TwoPi, limitRadius, 0.001, 64);
            capLimitMesh = rotate(capLimitMesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
            capLimitMesh = translate(capLimitMesh, position);
            Arrays.asList(capLimitMesh.getVertices()).forEach(v -> v.scaleAdd(limitPositionOnAxis, axis, v));
            faceMeshes.add(capLimitMesh.copy());

            Arrays.asList(capLimitMesh.getVertices()).forEach(v -> v.scaleAdd(-2.0 * limitPositionOnAxis, axis, v));
            faceMeshes.add(capLimitMesh);
         }
      }

      { // Edges
         Vector3D axisOrthogonal = newOrthogonalVector(axis);
         Point3D arcCenter = new Point3D();
         arcCenter.scaleAdd(radius, axisOrthogonal, topCenter);

         double capSphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * radius);
         Point3D capSphereCenter = new Point3D();
         capSphereCenter.scaleAdd(-capSphereOffset, axis, topCenter);

         double sideSphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, length);
         Point3D sideSphereCenter = new Point3D();
         sideSphereCenter.scaleAdd(-sideSphereOffset + radius, axisOrthogonal, position);

         UnitVector3D startDirection = new UnitVector3D();
         UnitVector3D endDirection = new UnitVector3D();

         startDirection.sub(arcCenter, sideSphereCenter);
         endDirection.sub(arcCenter, capSphereCenter);

         TriangleMesh3DDefinition arc = toArcPointsAndNormals(arcCenter, smallRadius, startDirection, endDirection, 32);
         TriangleMesh3DDefinition edgeMesh = applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false);
         faceMeshes.add(edgeMesh.copy());

         RotationMatrix flipRotation = new RotationMatrix();
         flipRotation.setAxisAngle(axisOrthogonal.getX(), axisOrthogonal.getY(), axisOrthogonal.getZ(), Math.PI);
         Arrays.asList(edgeMesh.getVertices()).forEach(v ->
         {
            v.sub(position);
            flipRotation.transform(v);
            v.add(position);
         });
         Arrays.asList(edgeMesh.getNormals()).forEach(n -> flipRotation.transform(n));
         faceMeshes.add(edgeMesh);
      }

      return new TriangleMesh3DDefinition[] {combine(true, false, faceMeshes), combine(true, false, edgeMeshes)};
   }

   public static TriangleMesh3DDefinition toSTPRamp3DMesh(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ, double smallRadius,
                                                          double largeRadius, boolean highlightLimits)
   {
      return combine(true, false, toSTPRamp3DMeshes(pose, sizeX, sizeY, sizeZ, smallRadius, largeRadius, highlightLimits));
   }

   public static TriangleMesh3DDefinition[] toSTPRamp3DMeshes(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ, double smallRadius,
                                                              double largeRadius, boolean highlightLimits)
   {
      Ramp3D stpRamp3D = new Ramp3D(sizeX, sizeY, sizeZ);
      if (pose != null)
         stpRamp3D.getPose().set(pose);
      return toSTPConvexPolytope3DMeshes(stpRamp3D.asConvexPolytope(), smallRadius, largeRadius, highlightLimits);
   }

   public static TriangleMesh3DDefinition toSTPConvexPolytope3DMesh(ConvexPolytope3DReadOnly convexPolytope, double smallRadius, double largeRadius,
                                                                    boolean highlightLimits)
   {
      return combine(true, false, toSTPConvexPolytope3DMeshes(convexPolytope, smallRadius, largeRadius, highlightLimits));
   }

   public static TriangleMesh3DDefinition[] toSTPConvexPolytope3DMeshes(ConvexPolytope3DReadOnly convexPolytope, double smallRadius, double largeRadius,
                                                                        boolean highlightLimits)
   {
      List<TriangleMesh3DDefinition> faceMeshes = new ArrayList<>();
      List<TriangleMesh3DDefinition> edgeMeshes = new ArrayList<>();
      List<TriangleMesh3DDefinition> vertexMeshes = new ArrayList<>();

      for (int faceIndex = 0; faceIndex < convexPolytope.getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = convexPolytope.getFace(faceIndex);
         // Produce faces' big sphere mesh
         faceMeshes.addAll(toFaceSpheres(face, largeRadius, smallRadius, highlightLimits));
         // Produce faces' inner-tori meshes (only for faces that are non-cyclic)
         edgeMeshes.addAll(toFaceInnerTori(face, largeRadius, smallRadius));
      }

      Set<HalfEdge3DReadOnly> processedHalfEdgeSet = new HashSet<>();

      for (int edgeIndex = 0; edgeIndex < convexPolytope.getNumberOfHalfEdges(); edgeIndex++)
      { // Building the edges' torus
         HalfEdge3DReadOnly halfEdge = convexPolytope.getHalfEdge(edgeIndex);
         if (processedHalfEdgeSet.contains(halfEdge.getTwin()))
            continue;

         processedHalfEdgeSet.add(halfEdge);
         edgeMeshes.add(toHalfEdgeTorus(halfEdge, largeRadius, smallRadius));
      }

      for (int vertexIndex = 0; vertexIndex < convexPolytope.getNumberOfVertices(); vertexIndex++)
      { // Building the vertices' small sphere
         Vertex3DReadOnly vertex = convexPolytope.getVertex(vertexIndex);
         vertexMeshes.addAll(toVertexSphere(vertex, largeRadius, smallRadius, false));
      }

      return new TriangleMesh3DDefinition[] {combine(true, false, faceMeshes), combine(true, false, edgeMeshes), combine(true, false, vertexMeshes)};
   }

   public static List<TriangleMesh3DDefinition> toFaceSpheres(Face3DReadOnly face, double largeRadius, double smallRadius, boolean highlightLimits)
   {
      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();
      boolean isFaceCyclicPolygon = isFaceCyclicPolygon(face, largeRadius, smallRadius);

      int startIndex = computeFaceStartIndex(face);
      Vertex3DReadOnly v0 = face.getVertex(startIndex);

      for (int indexOffset = 1; indexOffset < face.getNumberOfEdges() - 1; indexOffset++)
      {
         Vertex3DReadOnly v1 = face.getVertex((startIndex + indexOffset) % face.getNumberOfEdges());
         Vertex3DReadOnly v2 = face.getVertex((startIndex + indexOffset + 1) % face.getNumberOfEdges());

         meshes.addAll(toFaceSubSphere(face, v0, v1, v2, largeRadius, smallRadius, highlightLimits && !isFaceCyclicPolygon));
      }

      if (highlightLimits && isFaceCyclicPolygon)
      {
         meshes.addAll(toCyclicFaceSphereLimits(face, largeRadius, smallRadius, 0.001));
      }

      return meshes;
   }

   private static int computeFaceStartIndex(Face3DReadOnly face)
   {
      int startIndex = 0;
      double maxDistanceSquared = 0.0;

      for (int i = 0; i < face.getNumberOfEdges(); i++)
      {
         Vertex3DReadOnly v0 = face.getVertex(i);

         for (int j = 0; j < face.getNumberOfEdges(); j++)
         {
            Vertex3DReadOnly v1 = face.getVertex(j);

            double distanceSquared = v0.distanceSquared(v1);

            if (distanceSquared > maxDistanceSquared)
            {
               startIndex = i;
               maxDistanceSquared = distanceSquared;
            }
         }
      }

      return startIndex;
   }

   private static boolean isFaceCyclicPolygon(Face3DReadOnly face, double largeRadius, double smallRadius)
   {
      if (face.getNumberOfEdges() <= 3)
         return true;

      Point3D sphereCenter = new Point3D();
      Vertex3DReadOnly v0 = face.getVertex(0);
      Vertex3DReadOnly v1 = face.getVertex(1);
      Vertex3DReadOnly v2 = face.getVertex(2);
      double radius = largeRadius - smallRadius;
      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, radius, sphereCenter);
      double radiusSquared = EuclidCoreTools.square(radius);

      for (int vertexIndex = 3; vertexIndex < face.getNumberOfEdges(); vertexIndex++)
      {
         double distanceSquared = face.getVertex(vertexIndex).distanceSquared(sphereCenter);
         if (!EuclidCoreTools.epsilonEquals(radiusSquared, distanceSquared, 1.0e-12))
            return false;
      }

      return true;
   }

   public static List<TriangleMesh3DDefinition> toFaceSubSphere(Face3DReadOnly owner, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2,
                                                                double largeRadius, double smallRadius, boolean highlightLimits)
   {
      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();

      Point3D sphereCenter = new Point3D();
      Vector3D limitA = new Vector3D();
      Vector3D limitB = new Vector3D();
      Vector3D limitC = new Vector3D();

      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, largeRadius - smallRadius, sphereCenter);

      limitA.sub(v0, sphereCenter);
      limitB.sub(v1, sphereCenter);
      limitC.sub(v2, sphereCenter);

      meshes.add(toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, largeRadius, 32));

      if (highlightLimits)
         meshes.addAll(toFaceSubSphereLimits(owner, v0, v1, v2, largeRadius, smallRadius, 0.001));

      return meshes;
   }

   public static List<TriangleMesh3DDefinition> toCyclicFaceSphereLimits(Face3DReadOnly face, double largeRadius, double smallRadius, double lineThickness)
   {
      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();

      Point3D arcCenter = new Point3D();
      Vector3D arcNormal = new Vector3D();
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Point3D endpoint = new Point3D();

      Vertex3DReadOnly v0 = face.getVertex(0);
      Vertex3DReadOnly v1 = face.getVertex(1);
      Vertex3DReadOnly v2 = face.getVertex(2);
      double radius = largeRadius - smallRadius;
      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, radius, arcCenter);

      for (int edgeIndex = 0; edgeIndex < face.getNumberOfEdges(); edgeIndex++)
      {
         // Creates arcs to highlight the limits of the big spheres.
         HalfEdge3DReadOnly edge = face.getEdge(edgeIndex);
         EuclidGeometryTools.normal3DFromThreePoint3Ds(arcCenter, edge.getFirstEndpoint(), edge.getSecondEndpoint(), arcNormal);
         startDirection.sub(edge.getFirstEndpoint(), arcCenter);
         endDirection.sub(edge.getSecondEndpoint(), arcCenter);
         meshes.addAll(toSegmentedLine3DMesh(arcCenter, arcNormal, largeRadius, lineThickness, startDirection, startDirection.angle(endDirection), 32, 8));
         endpoint.scaleAdd(largeRadius / startDirection.length(), startDirection, arcCenter);
         meshes.add(translate(Sphere(lineThickness, 8, 8), endpoint));
      }

      return meshes;
   }

   private static List<TriangleMesh3DDefinition> toFaceSubSphereLimits(Face3DReadOnly owner, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2,
                                                                       double largeRadius, double smallRadius, double lineThickness)
   {
      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();

      Point3D arcCenter = new Point3D();
      Vector3D arcNormal = new Vector3D();
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Point3D endpoint = new Point3D();

      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, largeRadius - smallRadius, arcCenter);

      Vertex3DReadOnly[] vertices = {v0, v1, v2};

      for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
      {
         // Creates arcs to highlight the limits of the big spheres.
         Vertex3DReadOnly start = vertices[vertexIndex];
         Vertex3DReadOnly end = vertices[(vertexIndex + 1) % 3];
         EuclidGeometryTools.normal3DFromThreePoint3Ds(arcCenter, start, end, arcNormal);
         startDirection.sub(start, arcCenter);
         endDirection.sub(end, arcCenter);
         meshes.addAll(toSegmentedLine3DMesh(arcCenter, arcNormal, largeRadius, lineThickness, startDirection, startDirection.angle(endDirection), 32, 8));
         endpoint.scaleAdd(largeRadius / startDirection.length(), startDirection, arcCenter);
         meshes.add(translate(Sphere(lineThickness, 8, 8), endpoint));
      }

      return meshes;
   }

   public static List<TriangleMesh3DDefinition> toFaceInnerTori(Face3DReadOnly face, double largeRadius, double smallRadius)
   {
      if (isFaceCyclicPolygon(face, largeRadius, smallRadius))
         return Collections.emptyList();

      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();

      Point3D prevTriangleSphere = new Point3D();
      Point3D nextTriangleSphere = new Point3D();
      Vector3D prevSphereToEdge = new Vector3D();
      Vector3D nextSphereToEdge = new Vector3D();
      Point3D edgeCenter = new Point3D();
      UnitVector3D edgeAxis = new UnitVector3D();
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();

      int startIndex = computeFaceStartIndex(face);
      Vertex3DReadOnly v0 = face.getVertex(startIndex);

      for (int indexOffset = 2; indexOffset < face.getNumberOfEdges() - 1; indexOffset++)
      {
         Vertex3DReadOnly v1Prev = face.getVertex((startIndex + indexOffset - 1) % face.getNumberOfEdges());
         Vertex3DReadOnly v2Prev = face.getVertex((startIndex + indexOffset) % face.getNumberOfEdges());
         Vertex3DReadOnly v1Next = v2Prev;
         Vertex3DReadOnly v2Next = face.getVertex((startIndex + indexOffset + 1) % face.getNumberOfEdges());

         EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1Prev, v2Prev, largeRadius - smallRadius, prevTriangleSphere);
         EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1Next, v2Next, largeRadius - smallRadius, nextTriangleSphere);

         edgeCenter.add(v0, v2Prev);
         edgeCenter.scale(0.5);

         prevSphereToEdge.sub(edgeCenter, prevTriangleSphere);
         nextSphereToEdge.sub(edgeCenter, nextTriangleSphere);
         edgeAxis.sub(v2Prev, v0);
         edgeAxis.negate();
         double endRevolutionAngle = prevSphereToEdge.angle(nextSphereToEdge);

         startDirection.sub(v2Prev, nextTriangleSphere);
         endDirection.sub(v0, nextTriangleSphere);
         TriangleMesh3DDefinition arcData = toArcPointsAndNormals(nextTriangleSphere, largeRadius, startDirection, endDirection, 32);

         meshes.add(applyRevolution(arcData, edgeCenter, edgeAxis, 0.0, endRevolutionAngle, 16, false));
      }

      return meshes;
   }

   public static TriangleMesh3DDefinition toHalfEdgeTorus(HalfEdge3DReadOnly halfEdge, double largeRadius, double smallRadius)
   {
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Vector3D sphereToEdgeA = new Vector3D();
      Vector3D sphereToEdgeB = new Vector3D();

      Point3D neighborSphereCenterA = computeNeighborFaceSubSphereCenter(halfEdge, largeRadius, smallRadius);
      Point3D neighborSphereCenterB = computeNeighborFaceSubSphereCenter(halfEdge.getTwin(), largeRadius, smallRadius);

      sphereToEdgeA.sub(halfEdge.midpoint(), neighborSphereCenterA);
      sphereToEdgeB.sub(halfEdge.midpoint(), neighborSphereCenterB);
      Vector3DBasics revolutionAxis = halfEdge.getDirection(true);
      revolutionAxis.negate();
      double endRevolutionAngle = sphereToEdgeA.angle(sphereToEdgeB);

      startDirection.sub(halfEdge.getDestination(), neighborSphereCenterA);
      endDirection.sub(halfEdge.getOrigin(), neighborSphereCenterA);
      TriangleMesh3DDefinition arcData = toArcPointsAndNormals(neighborSphereCenterA, largeRadius, startDirection, endDirection, 32);

      return applyRevolution(arcData, halfEdge.midpoint(), revolutionAxis, 0.0, endRevolutionAngle, 16, false);
   }

   private static Point3D computeNeighborFaceSubSphereCenter(HalfEdge3DReadOnly edge, double largeRadius, double smallRadius)
   {
      Point3D neighorSubSphereCenter = new Point3D();

      Face3DReadOnly neighbor = edge.getFace();
      int edgeIndex = neighbor.getEdges().indexOf(edge);
      int neighborStartIndex = computeFaceStartIndex(neighbor);

      Vertex3DReadOnly v0 = neighbor.getVertex(neighborStartIndex);
      Vertex3DReadOnly v1;
      Vertex3DReadOnly v2;

      if (edgeIndex == neighborStartIndex)
      {
         v1 = neighbor.getVertex((neighborStartIndex + 1) % neighbor.getNumberOfEdges());
         v2 = neighbor.getVertex((neighborStartIndex + 2) % neighbor.getNumberOfEdges());
      }
      else
      {
         int neighborLastIndex = neighborStartIndex - 1;
         if (neighborLastIndex < 0)
            neighborLastIndex += neighbor.getNumberOfEdges();

         if (edgeIndex == neighborLastIndex)
         {
            int neighborSecondToLastIndex = neighborStartIndex - 2;
            if (neighborSecondToLastIndex < 0)
               neighborSecondToLastIndex += neighbor.getNumberOfEdges();
            v1 = neighbor.getVertex(neighborSecondToLastIndex);
            v2 = neighbor.getVertex(neighborLastIndex);
         }
         else
         {
            v1 = edge.getOrigin();
            v2 = edge.getDestination();
         }
      }

      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, largeRadius - smallRadius, neighorSubSphereCenter);

      return neighorSubSphereCenter;
   }

   public static List<TriangleMesh3DDefinition> toVertexSphere(Vertex3DReadOnly vertex, double largeRadius, double smallRadius, boolean addVerticesMesh)
   {
      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();

      for (int edgeIndex = 0; edgeIndex < vertex.getNumberOfAssociatedEdges(); edgeIndex++)
      {
         meshes.add(toVertexPartialSphere(vertex, vertex.getAssociatedEdge(edgeIndex), largeRadius, smallRadius, addVerticesMesh));
         meshes.addAll(toVertexPartialSpheres(vertex, vertex.getAssociatedEdge(edgeIndex).getFace(), largeRadius, smallRadius, addVerticesMesh));
      }
      return meshes;
   }

   public static TriangleMesh3DDefinition toVertexPartialSphere(Vertex3DReadOnly vertex, HalfEdge3DReadOnly associatedEdge, double largeRadius,
                                                                double smallRadius, boolean addVerticesMesh)
   {
      Vector3D limitC = new Vector3D();
      for (int faceIndex = 0; faceIndex < vertex.getNumberOfAssociatedEdges(); faceIndex++)
      {
         limitC.add(directionNeighborSubSphereToVertex(vertex, vertex.getAssociatedEdge(faceIndex).getFace(), largeRadius, smallRadius));
      }
      limitC.normalize();

      return toVertexPartialSphere(vertex,
                                   limitC,
                                   associatedEdge.midpoint(),
                                   associatedEdge.length(),
                                   computeNeighborFaceSubSphereCenter(associatedEdge.getTwin(), largeRadius, smallRadius),
                                   computeNeighborFaceSubSphereCenter(associatedEdge, largeRadius, smallRadius),
                                   largeRadius,
                                   smallRadius,
                                   addVerticesMesh);
   }

   public static List<TriangleMesh3DDefinition> toVertexPartialSpheres(Vertex3DReadOnly vertex, Face3DReadOnly neighbor, double largeRadius, double smallRadius,
                                                                       boolean addVerticesMesh)
   {
      if (isFaceCyclicPolygon(neighbor, largeRadius, smallRadius))
         return Collections.emptyList();

      Vector3D limitC = new Vector3D();
      for (int faceIndex = 0; faceIndex < vertex.getNumberOfAssociatedEdges(); faceIndex++)
      {
         limitC.add(directionNeighborSubSphereToVertex(vertex, vertex.getAssociatedEdge(faceIndex).getFace(), largeRadius, smallRadius));
      }
      limitC.normalize();

      List<TriangleMesh3DDefinition> meshes = new ArrayList<>();

      Point3D prevTriangleSphere = new Point3D();
      Point3D nextTriangleSphere = new Point3D();
      Point3D edgeCenter = new Point3D();

      int startIndex = computeFaceStartIndex(neighbor);
      Vertex3DReadOnly v0 = neighbor.getVertex(startIndex);

      for (int indexOffset = 2; indexOffset < neighbor.getNumberOfEdges() - 1; indexOffset++)
      {
         Vertex3DReadOnly v1Prev = neighbor.getVertex((startIndex + indexOffset - 1) % neighbor.getNumberOfEdges());
         Vertex3DReadOnly v2Prev = neighbor.getVertex((startIndex + indexOffset) % neighbor.getNumberOfEdges());
         Vertex3DReadOnly v1Next = v2Prev;
         Vertex3DReadOnly v2Next = neighbor.getVertex((startIndex + indexOffset + 1) % neighbor.getNumberOfEdges());

         EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1Prev, v2Prev, largeRadius - smallRadius, prevTriangleSphere);
         EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1Next, v2Next, largeRadius - smallRadius, nextTriangleSphere);

         edgeCenter.add(v0, v2Prev);
         edgeCenter.scale(0.5);

         meshes.add(toVertexPartialSphere(vertex,
                                          limitC,
                                          edgeCenter,
                                          v0.distance(v2Prev),
                                          prevTriangleSphere,
                                          nextTriangleSphere,
                                          largeRadius,
                                          smallRadius,
                                          addVerticesMesh));
      }

      return meshes;
   }

   public static TriangleMesh3DDefinition toVertexPartialSphere(Vertex3DReadOnly vertex, Vector3DReadOnly limitC, Point3DReadOnly commonEdgeCenter,
                                                                double commonEdgeLength, Point3DReadOnly sphereA, Point3DReadOnly sphereB, double largeRadius,
                                                                double smallRadius, boolean addVerticesMesh)
   {
      Vector3D sphereAToEdge = new Vector3D();
      sphereAToEdge.sub(commonEdgeCenter, sphereA);
      sphereAToEdge.normalize();
      Vector3D sphereBToEdge = new Vector3D();
      sphereBToEdge.sub(commonEdgeCenter, sphereB);
      sphereBToEdge.normalize();

      Vector3D testWinding = new Vector3D();
      testWinding.cross(sphereAToEdge, sphereBToEdge);
      boolean flip = testWinding.dot(limitC) > 0.0;

      double radius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, commonEdgeLength);
      Vector3D sphereToEdge = new Vector3D();
      Point3D sphereCenter = new Point3D();
      Vector3D limitAB = new Vector3D();

      DoubleFunction<Vector3D> limitABFunction = alpha ->
      {
         if (flip)
            alpha = 1.0 - alpha;
         sphereToEdge.interpolate(sphereAToEdge, sphereBToEdge, alpha);
         sphereToEdge.scale(radius / sphereToEdge.length());
         sphereCenter.sub(commonEdgeCenter, sphereToEdge);
         limitAB.sub(vertex, sphereCenter);
         limitAB.normalize();
         return limitAB;
      };

      return toPartialSphereMesh(vertex, limitABFunction, limitC, smallRadius, 16, false);
   }

   private static Vector3DReadOnly directionNeighborSubSphereToVertex(Vertex3DReadOnly vertex, Face3DReadOnly neighbor, double largeRadius, double smallRadius)
   {
      int edgeIndex = neighbor.getVertices().indexOf(vertex);
      Vector3D direction = new Vector3D();
      direction.sub(vertex, computeNeighborFaceSubSphereCenter(neighbor.getEdge(edgeIndex), largeRadius, smallRadius));
      return direction;
   }

   public static List<TriangleMesh3DDefinition> toSegmentedLine3DMesh(Point3DReadOnly arcCenter, Vector3DReadOnly arcNormal, double arcRadius, double thickness,
                                                                      Vector3DReadOnly startDirection, double angleSpan, int resolution, int radialResolution)
   {
      SegmentedLine3DTriangleMeshFactory generator = new SegmentedLine3DTriangleMeshFactory(resolution, radialResolution);

      AxisAngle axisAngle = new AxisAngle(arcNormal, 0.0);
      Vector3D direction = new Vector3D();
      Point3D[] points = new Point3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         axisAngle.setAngle(angleSpan * i / (resolution - 1.0));
         axisAngle.transform(startDirection, direction);
         direction.normalize();

         Point3D point = new Point3D();
         point.scaleAdd(arcRadius, direction, arcCenter);
         points[i] = point;
      }

      generator.setLineRadius(thickness);
      generator.compute(points);
      return Arrays.asList(generator.getTriangleMesh3DDefinitions());
   }

   public static TriangleMesh3DDefinition toPartialSphereMesh(Point3DReadOnly sphereCenter, Tuple3DReadOnly limitA, Tuple3DReadOnly limitB,
                                                              Tuple3DReadOnly limitC, double sphereRadius, int resolution)
   {
      return toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, sphereRadius, resolution, false);
   }

   public static TriangleMesh3DDefinition toPartialSphereMesh(Point3DReadOnly sphereCenter, Tuple3DReadOnly limitA, Tuple3DReadOnly limitB,
                                                              Tuple3DReadOnly limitC, double sphereRadius, int resolution, boolean addVerticesMesh)
   {
      return toPartialSphereMesh(sphereCenter,
                                 alpha -> interpolateVector3D(limitA, limitB, alpha),
                                 alpha -> interpolateVector3D(limitB, limitC, alpha),
                                 alpha -> interpolateVector3D(limitC, limitA, alpha),
                                 sphereRadius,
                                 resolution,
                                 addVerticesMesh);
   }

   public static TriangleMesh3DDefinition toPartialSphereMesh(Point3DReadOnly sphereCenter, DoubleFunction<? extends Tuple3DReadOnly> limitABFunction,
                                                              Tuple3DReadOnly limitC, double sphereRadius, int resolution, boolean addVerticesMesh)
   {
      Vector3D limitA = new Vector3D(limitABFunction.apply(0.0));
      Vector3D limitB = new Vector3D(limitABFunction.apply(1.0));
      limitA.normalize();
      limitB.normalize();

      return toPartialSphereMesh(sphereCenter,
                                 limitABFunction,
                                 alpha -> interpolateVector3D(limitB, limitC, alpha),
                                 alpha -> interpolateVector3D(limitC, limitA, alpha),
                                 sphereRadius,
                                 resolution,
                                 addVerticesMesh);
   }

   public static TriangleMesh3DDefinition toPartialSphereMesh(Point3DReadOnly sphereCenter, DoubleFunction<? extends Tuple3DReadOnly> limitABFunction,
                                                              DoubleFunction<? extends Tuple3DReadOnly> limitBCFunction,
                                                              DoubleFunction<? extends Tuple3DReadOnly> limitCAFunction, double sphereRadius, int resolution,
                                                              boolean addVerticesMesh)
   {
      List<Point3D32> points = new ArrayList<>();
      List<Vector3D32> normals = new ArrayList<>();
      List<Point2D32> textPoints = new ArrayList<>();

      Point3D limitAB = new Point3D();

      for (int longitude = 0; longitude < resolution - 1; longitude++)
      {
         double longitudeAlpha = longitude / (resolution - 1.0);
         int latitudeResolution = (int) Math.round(EuclidCoreTools.interpolate(resolution, 1, longitudeAlpha));

         { // latitude = 0, point is on AC limit
            Vector3D32 direction = new Vector3D32();
            direction.set(limitCAFunction.apply(1.0 - longitudeAlpha));
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new Point2D32((float) longitudeAlpha, 0.0f));
         }

         for (int latitude = 1; latitude < latitudeResolution - 1; latitude++)
         {
            double latitudeAlpha = latitude / (latitudeResolution - 1.0);
            limitAB.set(limitABFunction.apply(latitudeAlpha));
            Vector3D32 direction = new Vector3D32();
            direction.interpolate(limitAB, limitCAFunction.apply(0.0), longitudeAlpha);
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new Point2D32((float) longitudeAlpha, (float) latitudeAlpha));
         }

         { // latitude = latitudeResolution - 1, point is on BC limit
            Vector3D32 direction = new Vector3D32();
            direction.set(limitBCFunction.apply(longitudeAlpha));
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new Point2D32((float) longitudeAlpha, 1.0f));
         }
      }

      { // Last vertex lies on limitC
         Vector3D32 direction = new Vector3D32();
         direction.set(limitCAFunction.apply(0.0));
         direction.normalize();
         Point3D32 point = new Point3D32();
         point.scaleAdd(sphereRadius, direction, sphereCenter);
         points.add(point);
         normals.add(direction);
         textPoints.add(new Point2D32(1.0f, 1.0f));
      }

      TIntArrayList triangleIndices = new TIntArrayList();

      int nextLatitudeResolution = resolution;
      int longitudeStartIndex = 0;

      for (int longitude = 0; longitude < resolution - 1; longitude++)
      {
         int latitudeResolution = nextLatitudeResolution;
         nextLatitudeResolution = (int) Math.round(EuclidCoreTools.interpolate(resolution, 1, (longitude + 1.0) / (resolution - 1.0)));
         int nextLongitudeStartIndex = longitudeStartIndex + latitudeResolution;

         for (int latitude = 0; latitude < latitudeResolution - 1; latitude++)
         {
            if (latitude < nextLatitudeResolution)
            {
               triangleIndices.add(longitudeStartIndex + latitude);
               triangleIndices.add(nextLongitudeStartIndex + latitude);
               triangleIndices.add(longitudeStartIndex + latitude + 1);
            }

            if (latitude < nextLatitudeResolution - 1)
            {
               triangleIndices.add(nextLongitudeStartIndex + latitude);
               triangleIndices.add(nextLongitudeStartIndex + latitude + 1);
               triangleIndices.add(longitudeStartIndex + latitude + 1);
            }
         }

         longitudeStartIndex += latitudeResolution;
      }

      TriangleMesh3DDefinition partialSphereMesh = new TriangleMesh3DDefinition(points.toArray(new Point3D32[0]),
                                                                                textPoints.toArray(new Point2D32[0]),
                                                                                normals.toArray(new Vector3D32[0]),
                                                                                triangleIndices.toArray());
      if (!addVerticesMesh)
      {
         return partialSphereMesh;
      }
      else
      {
         TriangleMesh3DDefinition[] meshes = new TriangleMesh3DDefinition[1 + points.size()];
         meshes[0] = partialSphereMesh;
         for (int i = 0; i < points.size(); i++)
            meshes[i + 1] = translate(Tetrahedron(0.005), points.get(i));
         return combine(true, false, meshes);
      }
   }

   public static TriangleMesh3DDefinition toArcPointsAndNormals(Point3DReadOnly arcPosition, double arcRadius, Vector3DReadOnly startDirection,
                                                                Vector3DReadOnly endDirection, int resolution)
   {
      Point3D32[] points = new Point3D32[resolution];
      Vector3D32[] normals = new Vector3D32[resolution];

      for (int i = 0; i < resolution; i++)
      {
         double alpha = i / (resolution - 1.0);
         Vector3D32 direction = new Vector3D32();
         direction.interpolate(startDirection, endDirection, alpha);
         direction.normalize();

         Point3D32 point = new Point3D32();
         point.scaleAdd(arcRadius, direction, arcPosition);

         points[i] = point;
         normals[i] = direction;
      }

      return new TriangleMesh3DDefinition(points, null, normals, null);
   }

   public static TriangleMesh3DDefinition applyRevolution(TriangleMesh3DDefinition subMesh, Point3DReadOnly rotationCenter, Vector3DReadOnly rotationAxis,
                                                          double startAngle, double endAngle, int resolution, boolean addVerticesMesh)
   {
      int subMeshSize = subMesh.getVertices().length;
      Point3D32[] points = new Point3D32[resolution * subMeshSize];
      Vector3D32[] normals = new Vector3D32[resolution * subMeshSize];
      Point2D32[] textPoints = new Point2D32[resolution * subMeshSize];

      AxisAngle rotation = new AxisAngle();
      rotation.getAxis().set(rotationAxis);

      for (int revIndex = 0; revIndex < resolution; revIndex++)
      {
         double angle = EuclidCoreTools.interpolate(startAngle, endAngle, revIndex / (resolution - 1.0));
         rotation.setAngle(angle);

         for (int meshIndex = 0; meshIndex < subMeshSize; meshIndex++)
         {
            Point3D32 point = new Point3D32(subMesh.getVertices()[meshIndex]);
            Vector3D32 normal = new Vector3D32(subMesh.getNormals()[meshIndex]);

            point.sub(rotationCenter);

            rotation.transform(point);
            rotation.transform(normal);

            point.add(rotationCenter);

            points[revIndex * subMeshSize + meshIndex] = point;
            normals[revIndex * subMeshSize + meshIndex] = normal;
            textPoints[revIndex * subMeshSize + meshIndex] = new Point2D32(revIndex / (resolution - 1.0f), meshIndex / (subMeshSize - 1.0f));
         }
      }

      int numberOfTriangles = 2 * resolution * subMeshSize;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      for (int revIndex = 0; revIndex < resolution - 1; revIndex++)
      {
         for (int meshIndex = 0; meshIndex < subMeshSize - 1; meshIndex++)
         {
            int nextRevIndex = revIndex + 1;
            int nextMeshIndex = meshIndex + 1;

            triangleIndices[index++] = nextRevIndex * subMeshSize + meshIndex;
            triangleIndices[index++] = nextRevIndex * subMeshSize + nextMeshIndex;
            triangleIndices[index++] = revIndex * subMeshSize + nextMeshIndex;

            triangleIndices[index++] = nextRevIndex * subMeshSize + meshIndex;
            triangleIndices[index++] = revIndex * subMeshSize + nextMeshIndex;
            triangleIndices[index++] = revIndex * subMeshSize + meshIndex;
         }
      }

      TriangleMesh3DDefinition partialSphereMesh = new TriangleMesh3DDefinition(points, textPoints, normals, triangleIndices);

      if (!addVerticesMesh)
      {
         return partialSphereMesh;
      }
      else
      {
         TriangleMesh3DDefinition[] meshes = new TriangleMesh3DDefinition[1 + points.length];
         meshes[0] = partialSphereMesh;
         for (int i = 0; i < points.length; i++)
            meshes[i + 1] = translate(Tetrahedron(0.005), points[i]);
         return combine(true, false, meshes);
      }
   }

   public static TriangleMesh3DDefinition rotate(TriangleMesh3DDefinition in, Orientation3DReadOnly rotation)
   {
      for (Point3D32 vertex : in.getVertices())
         rotation.transform(vertex);
      for (Vector3D32 normal : in.getNormals())
         rotation.transform(normal);
      return in;
   }

   public static TriangleMesh3DDefinition translate(TriangleMesh3DDefinition in, Tuple3DReadOnly translation)
   {
      for (Point3D32 vertex : in.getVertices())
         vertex.add(translation);
      return in;
   }

   public static TriangleMesh3DDefinition combine(boolean adjustTriangleIndices, boolean deepCopy, TriangleMesh3DDefinition... definitions)
   {
      return combine(adjustTriangleIndices, deepCopy, Arrays.asList(definitions));
   }

   public static TriangleMesh3DDefinition combine(boolean adjustTriangleIndices, boolean deepCopy, Collection<TriangleMesh3DDefinition> definitions)
   {
      int outVertexSize = 0;
      int outTextureSize = 0;
      int outNormalSize = 0;
      int outIndexSize = 0;

      for (TriangleMesh3DDefinition definition : definitions)
      {
         outVertexSize += definition.getVertices() != null ? definition.getVertices().length : 0;
         outTextureSize += definition.getTextures() != null ? definition.getTextures().length : 0;
         outNormalSize += definition.getNormals() != null ? definition.getNormals().length : 0;
         outIndexSize += definition.getTriangleIndices() != null ? definition.getTriangleIndices().length : 0;
      }

      Point3D32[] outVertices = new Point3D32[outVertexSize];
      Point2D32[] outTextures = new Point2D32[outTextureSize];
      Vector3D32[] outNormals = new Vector3D32[outNormalSize];
      int[] outIndices = new int[outIndexSize];

      int vertexIndex = 0;
      int textureIndex = 0;
      int normalIndex = 0;
      int indexIndex = 0;

      for (TriangleMesh3DDefinition definition : definitions)
      {
         Point3D32[] inVertices = definition.getVertices();
         Point2D32[] inTextures = definition.getTextures();
         Vector3D32[] inNormals = definition.getNormals();
         int[] inIndices = definition.getTriangleIndices();

         if (inVertices != null)
         {
            if (deepCopy)
            {
               for (Point3D32 vertex : inVertices)
                  outVertices[vertexIndex++] = new Point3D32(vertex);
            }
            else
            {
               System.arraycopy(inVertices, 0, outVertices, vertexIndex, inVertices.length);
               vertexIndex += inVertices.length;
            }
         }

         if (inTextures != null)
         {
            if (deepCopy)
            {
               for (Point2D32 texture : inTextures)
                  outTextures[textureIndex++] = new Point2D32(texture);
            }
            else
            {
               System.arraycopy(inTextures, 0, outTextures, textureIndex, inTextures.length);
               textureIndex += inTextures.length;
            }
         }

         if (inNormals != null)
         {
            if (deepCopy)
            {
               for (Vector3D32 normal : inNormals)
                  outNormals[normalIndex++] = new Vector3D32(normal);
            }
            else
            {
               System.arraycopy(inNormals, 0, outNormals, normalIndex, inNormals.length);
               normalIndex += inNormals.length;
            }
         }

         if (inIndices != null)
         {
            System.arraycopy(inIndices, 0, outIndices, indexIndex, inIndices.length);
            indexIndex += inIndices.length;
         }
      }

      if (adjustTriangleIndices && definitions.size() > 1)
      {
         Iterator<TriangleMesh3DDefinition> iterator = definitions.iterator();

         TriangleMesh3DDefinition definition = iterator.next();
         int shift = definition.getVertices().length;
         int startIndex = definition.getTriangleIndices().length;

         while (iterator.hasNext())
         {
            definition = iterator.next();
            int endIndex = startIndex + definition.getTriangleIndices().length;

            for (int i = startIndex; i < endIndex; i++)
            {
               outIndices[i] += shift;
            }

            shift += definition.getVertices().length;
            startIndex = endIndex;
         }
      }

      return new TriangleMesh3DDefinition(outVertices, outTextures, outNormals, outIndices);
   }

   public static Vector3D newOrthogonalVector(Vector3DReadOnly referenceVector)
   {
      Vector3D orthogonal = new Vector3D();

      // Purposefully picking a large tolerance to ensure sanity of the cross-product.
      if (Math.abs(referenceVector.getY()) > 0.1 || Math.abs(referenceVector.getZ()) > 0.1)
         orthogonal.set(1.0, 0.0, 0.0);
      else
         orthogonal.set(0.0, 1.0, 0.0);

      orthogonal.cross(referenceVector);
      orthogonal.normalize();

      return orthogonal;
   }

   public static Vector3D interpolateVector3D(Tuple3DReadOnly tuple0, Tuple3DReadOnly tuplef, double alpha)
   {
      Vector3D interpolated = new Vector3D();
      interpolated.interpolate(tuple0, tuplef, alpha);
      return interpolated;
   }

   /**
    * Reverses the order of the elements in the specified array.
    * <p>
    * This method runs in linear time.
    * </p>
    *
    * @param array the array whose elements are to be reversed.
    */
   public static void reverse(Object[] array)
   {
      for (int i = 0, mid = array.length >> 1, j = array.length - 1; i < mid; i++, j--)
      {
         Object oldCoefficient_i = array[i];
         array[i] = array[j];
         array[j] = oldCoefficient_i;
      }
   }
}