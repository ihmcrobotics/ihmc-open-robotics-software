package us.ihmc.graphicsDescription;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;

/**
 * {@code Segmented3DLineMeshDataGenerator} generates an array of {@code MeshDataHolder}s that
 * describes a segmented line 3D that goes through 3D waypoints.
 * <p>
 * It was originally implemented to enabled elegant representation of 3D trajectories, see
 * {@link YoGraphicPolynomial3D}.
 * </p>
 * <p>
 * It has been optimized using YourKit for reducing computation time while guaranteeing free
 * allocation, i.e. no garbage generation such that it does not show up when tracking down garbage
 * generation in the controller.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public final class SegmentedLine3DMeshDataGenerator
{
   /**
    * The meshes for this segmented line 3D. Not that the meshes are recycled.
    */
   private final MeshDataHolder[] meshDataHolders;
   /**
    * The {@code circleTemplate} is used to reset the other circles without recomputing the circle
    * equation.
    */
   private final CircleVertices circleTemplate;
   private final CircleVertices[] circles;

   /**
    * Create a new mesh generator given the properties necessary to initialize the meshes.
    * <p>
    * The line radius is set to 1 meter but can be changed later via the method
    * {@link #setLineRadius(double)}.
    * </p>
    * 
    * @param numberOfWaypoints the number of waypoints this segmented line will have to go through.
    *           Necessary to evaluate the number of meshes necessary.
    * @param radialResolution refers to the quality of the cylinder rendering of the line section. A
    *           high value will result in a smooth circle section, while a low value result in a
    *           polygonized section.
    */
   public SegmentedLine3DMeshDataGenerator(int numberOfWaypoints, int radialResolution)
   {
      this(numberOfWaypoints, radialResolution, 1.0);
   }

   /**
    * Create a new mesh generator given the properties necessary to initialize the meshes and the
    * radius to use for the line.
    * 
    * @param numberOfWaypoints the number of waypoints this segmented line will have to go through.
    *           Necessary to evaluate the number of meshes necessary.
    * @param radialResolution refers to the quality of the cylinder rendering of the line section. A
    *           high value will result in a smooth circle section, while a low value result in a
    *           polygonized section.
    * @param radius the radius used when rendering the 3D line.
    */
   public SegmentedLine3DMeshDataGenerator(int numberOfWaypoints, int radialResolution, double radius)
   {
      circleTemplate = new CircleVertices(radialResolution);
      circleTemplate.setRadius(radius);
      circles = new CircleVertices[numberOfWaypoints];
      for (int i = 0; i < numberOfWaypoints; i++)
         circles[i] = new CircleVertices(radialResolution);

      meshDataHolders = createMeshDataHolders(circles);
   }

   private final Vector3D previousDirection = new Vector3D();

   /**
    * Update the meshes of this generator to represent a segmented line 3D that goes through the
    * given set of waypoints.
    * <p>
    * The resulting meshes can be obtained using {@link #getMeshDataHolders()}.
    * </p>
    * 
    * @param waypointPositions the positions through which the segmented line 3D has to go through.
    * @throws RuntimeException if {@code waypointPositions.length != this.getNumberOfWaypoints()}.
    */
   public void compute(Point3DReadOnly[] waypointPositions)
   {
      compute(waypointPositions, null);
   }

   /**
    * Update the meshes of this generator to represent a segmented line 3D that goes through the
    * given set of waypoints.
    * <p>
    * The resulting meshes can be obtained using {@link #getMeshDataHolders()}.
    * </p>
    * <p>
    * The given {@code waypointDirections} are used as the section normals of the line at the
    * waypoints. If it is {@code null}, it is computed internally based on the waypoint positions
    * and spacing.
    * </p>
    * 
    * @param waypointPositions the positions through which the segmented line 3D has to go through.
    * @param waypointDirections the positions through which the segmented line 3D has to go through.
    * @throws RuntimeException if {@code waypointPositions.length != this.getNumberOfWaypoints()} or
    * @throws RuntimeException if {@code waypointDirections != null} and that
    *            {@code waypointDirections.length != this.getNumberOfWaypoints()}.
    */
   public void compute(Point3DReadOnly[] waypointPositions, Vector3DReadOnly[] waypointDirections)
   {
      if (waypointPositions.length != circles.length)
         throw new RuntimeException("Unexpected array size. Expected: " + circles.length + ", but was: " + waypointPositions.length);
      if (waypointDirections != null && waypointDirections.length != circles.length)
         throw new RuntimeException("Unexpected array size. Expected: " + circles.length + ", but was: " + waypointDirections.length);

      int sectionIndex = 0;

      previousDirection.set(0.0, 0.0, 1.0);

      for (sectionIndex = 0; sectionIndex < circles.length; sectionIndex++)
      {
         Vector3DReadOnly sectionDirection = computeNormalizedSectionDirection(sectionIndex, waypointPositions, waypointDirections);
         // Computing the rotation that is the closest from the rotation of the previous direction.
         // This trick allows to preserve the natural indexing of the base and top circles of each section.
         // If instead the minimum rotation from zUp to sectionDiretion was used, some of the section meshes would be messed up.
         computeRotation(previousDirection, sectionDirection, rotation);
         previousDirection.set(sectionDirection);

         CircleVertices currentCircle = circles[sectionIndex];
         currentCircle.set(circleTemplate); // Resetting the vertices and normals of the circle using the unmodified template.
         currentCircle.rotate(rotation);
         currentCircle.translate(waypointPositions[sectionIndex]); // Translation has to be done after the rotation of the vertices.

      }
   }

   /**
    * Changes the radius used when rendering the 3D line.
    * <p>
    * One of the compute methods has to be called before the change is effective on the output
    * meshes.
    * </p>
    * 
    * @param radius the new radius to be used when rendering the 3D line.
    */
   public void setLineRadius(double radius)
   {
      circleTemplate.setRadius(radius);
   }

   /**
    * Gets the number of waypoints the segmented line 3D goes through (including the origin and the
    * end of the line).
    * 
    * @return the number of waypoints.
    */
   public int getNumberOfWaypoints()
   {
      return circles.length;
   }

   /**
    * Gets the reference to the output meshes of this generator.
    * <p>
    * WARNING: the meshes are part of the internal memory of this generator and are updated when
    * calling one of the compute methods.
    * </p>
    * 
    * @return the reference to the output meshes of this generator.
    */
   public MeshDataHolder[] getMeshDataHolders()
   {
      return meshDataHolders;
   }

   /**
    * Gets the line radius currently used by this generator.
    * 
    * @return the line radius.
    */
   public double getLineRadius()
   {
      return circleTemplate.radius;
   }

   /**
    * Gets the radial resolution currently used by this generator.
    * 
    * @return the radial resolution.
    */
   public int getRadialResolution()
   {
      return circleTemplate.getNumberOfVertices();
   }

   /**
    * Creates the output {@code MeshDataHolder}s.
    * <p>
    * WARNING: the {@code MeshDataHolder[]} and {@code CircleMeshVertices[]} share the same
    * instances for the vertices and normals, such that this generator actually modifies the meshes
    * by simply updating the circles. This is for improved performance.
    * </p>
    * 
    * @param circles the circles from which the output meshes will be created. Not modified, but
    *           references to the vertices and normals are stored in the resulting meshes.
    * @return the array of {@code MeshDataHolder} that this generator will update.
    */
   private static MeshDataHolder[] createMeshDataHolders(CircleVertices[] circles)
   {
      int numberOfWaypoint = circles.length;

      MeshDataHolder[] meshDataHolders = new MeshDataHolder[numberOfWaypoint - 1];

      for (int waypointIndex = 0; waypointIndex < numberOfWaypoint - 1; waypointIndex++)
      {
         CircleVertices currentCircle = circles[waypointIndex];
         CircleVertices nextCircle = circles[waypointIndex + 1];
         int radialResolution = currentCircle.getNumberOfVertices();
         int numberOfVertices = 2 * radialResolution;

         TexCoord2f[] texturePoints = new TexCoord2f[numberOfVertices];
         Point3D32[] vertices = new Point3D32[numberOfVertices];
         Vector3D32[] vertexNormals = new Vector3D32[numberOfVertices];

         for (int i = 0; i < radialResolution; i++)
         {
            vertices[i] = currentCircle.vertices[i];
            vertexNormals[i] = currentCircle.normals[i];

            vertices[i + radialResolution] = nextCircle.vertices[i];
            vertexNormals[i + radialResolution] = nextCircle.normals[i];
         }

         int index = 0;
         int[] triangleIndices = new int[6 * radialResolution];

         for (int vertexIndex = 0; vertexIndex < radialResolution; vertexIndex++)
         { // The cylinder part
            int nextVertexIndex = (vertexIndex + 1) % radialResolution;
            // Lower triangle
            triangleIndices[index++] = vertexIndex;
            triangleIndices[index++] = nextVertexIndex;
            triangleIndices[index++] = vertexIndex + radialResolution;
            // Upper triangle
            triangleIndices[index++] = nextVertexIndex;
            triangleIndices[index++] = nextVertexIndex + radialResolution;
            triangleIndices[index++] = vertexIndex + radialResolution;
         }

         for (int i = 0; i < numberOfVertices; i++)
         {
            texturePoints[i] = new TexCoord2f();
         }
         meshDataHolders[waypointIndex] = new MeshDataHolder(vertices, texturePoints, triangleIndices, vertexNormals);
      }

      return meshDataHolders;
   }

   /** Using a Matrix3D instead of RotationMatrix to speed up calculation. */
   private final Matrix3D rotation = new Matrix3D();
   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();

   private void computeRotation(Vector3DReadOnly previousDirection, Vector3DReadOnly sectionDirection, Matrix3D rotationToPack)
   {
      xAxis.cross(previousDirection, sectionDirection);
      xAxis.normalize();
      yAxis.cross(sectionDirection, xAxis);
      rotationToPack.setColumn(0, xAxis);
      rotationToPack.setColumn(1, yAxis);
      rotationToPack.setColumn(2, sectionDirection);
   }

   private final Vector3D tempDirection = new Vector3D();
   private final Vector3D tempPreviousSegment = new Vector3D();
   private final Vector3D tempNextSegment = new Vector3D();

   private Vector3DReadOnly computeNormalizedSectionDirection(int sectionIndex, Point3DReadOnly[] sectionCenters, Vector3DReadOnly[] sectionDirections)
   {
      if (sectionDirections != null)
      {
         tempDirection.set(sectionDirections[sectionIndex]);
         double length = tempDirection.length();
         if (length > Epsilons.ONE_HUNDRED_MILLIONTH)
         {
            tempDirection.scale(1.0 / length);
            return tempDirection;
         }
      }

      if (sectionIndex == 0)
      {
         tempDirection.sub(sectionCenters[1], sectionCenters[0]);
      }
      else if (sectionIndex == sectionCenters.length - 1)
      {
         tempDirection.sub(sectionCenters[sectionCenters.length - 1], sectionCenters[sectionCenters.length - 2]);
      }
      else
      {
         Point3DReadOnly previousCenter = sectionCenters[sectionIndex - 1];
         Point3DReadOnly currentCenter = sectionCenters[sectionIndex];
         Point3DReadOnly nextCenter = sectionCenters[sectionIndex + 1];

         tempPreviousSegment.sub(currentCenter, previousCenter);
         tempNextSegment.sub(nextCenter, currentCenter);
         double previousLength = tempPreviousSegment.length();
         double nextLength = tempNextSegment.length();

         double alpha = nextLength / (previousLength + nextLength);
         tempDirection.interpolate(tempPreviousSegment, tempNextSegment, alpha);
      }

      tempDirection.normalize();
      return tempDirection;
   }

   private final static class CircleVertices
   {
      private final Point3D32[] vertices;
      private final Vector3D32[] normals;

      private double radius = 1.0;

      public CircleVertices(int numberOfVertices)
      {
         vertices = new Point3D32[numberOfVertices];
         normals = new Vector3D32[numberOfVertices];

         for (int i = 0; i < numberOfVertices; i++)
         {
            vertices[i] = new Point3D32();
            normals[i] = new Vector3D32();
         }

         for (int i = 0; i < numberOfVertices; i++)
         {
            double angle = i / (double) numberOfVertices * 2.0 * Math.PI;
            double ux = Math.cos(angle);
            double uy = Math.sin(angle);
            vertices[i].set(radius * ux, radius * uy, 0.0);
            normals[i].set(ux, uy, 0.0);
         }
      }

      public void setRadius(double radius)
      {
         double scale = radius / this.radius;
         for (Point3D32 vertex : vertices)
            vertex.scale(scale);
         this.radius = radius;
      }

      private void set(CircleVertices other)
      {
         int numberOfVertices = getNumberOfVertices();

         for (int i = 0; i < numberOfVertices; i++)
         {
            vertices[i].set(other.vertices[i]);
            normals[i].set(other.normals[i]);
         }
      }

      public void translate(Tuple3DReadOnly translation)
      {
         for (Point3D32 vertex : vertices)
            vertex.add(translation);
      }

      public void rotate(Matrix3DReadOnly rotationMatrix)
      {
         for (Point3D32 vertex : vertices)
         {
            double x = rotationMatrix.getM00() * vertex.getX() + rotationMatrix.getM01() * vertex.getY();
            double y = rotationMatrix.getM10() * vertex.getX() + rotationMatrix.getM11() * vertex.getY();
            double z = rotationMatrix.getM20() * vertex.getX() + rotationMatrix.getM21() * vertex.getY();
            vertex.set(x, y, z);
         }

         for (Vector3D32 normal : normals)
         {
            double x = rotationMatrix.getM00() * normal.getX() + rotationMatrix.getM01() * normal.getY();
            double y = rotationMatrix.getM10() * normal.getX() + rotationMatrix.getM11() * normal.getY();
            double z = rotationMatrix.getM20() * normal.getX() + rotationMatrix.getM21() * normal.getY();
            normal.set(x, y, z);
         }
      }

      public int getNumberOfVertices()
      {
         return vertices.length;
      }
   }
}
