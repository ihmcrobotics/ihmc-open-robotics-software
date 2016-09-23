package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Data structure for the Expanding Polytope Algorithm described in "Collision Detection in Interactive 3D Environments"
 * by Gino van den Bergen. Vertices and edges are index counterclockwise.
 *
 */
public class ExpandingPolytopeEntry implements Comparable<ExpandingPolytopeEntry>
{
   private final Point3d[] triangleVertices;
   private final Vector3d closestPointToOrigin = new Vector3d();
   private final double[] lambdas = new double[3];
   private double distanceToOriginKey;

   private final ExpandingPolytopeEntry[] adjacentTriangles = new ExpandingPolytopeEntry[3];
   private final int[] adjacentTriangleEdgeIndices = new int[3];

   private boolean obsolete = false;

   public Point3d getVertex(int index)
   {
      return triangleVertices[index];
   }

   public ExpandingPolytopeEntry(Point3d pointOne, Point3d pointTwo, Point3d pointThree)
   {
      triangleVertices = new Point3d[] {pointOne, pointTwo, pointThree};
      projectOriginOntoFace(pointOne, pointTwo, pointThree, closestPointToOrigin, lambdas);
      distanceToOriginKey = closestPointToOrigin.length();
      if (Double.isNaN(distanceToOriginKey)) throw new RuntimeException();
   }

   public boolean closestIsInternal()
   {
      for (int i=0; i<3; i++)
      {
         if (lambdas[i] < 0.0) return false;
         if (lambdas[i] > 1.0) return false;
      }

      return true;
   }

   public void setAdjacentTriangle(int thisTriangleEdgeIndex, ExpandingPolytopeEntry adjacentTriangleEntry, int adjacentTriangleEdgeIndex)
   {
      adjacentTriangles[thisTriangleEdgeIndex] = adjacentTriangleEntry;
      adjacentTriangleEdgeIndices[thisTriangleEdgeIndex] = adjacentTriangleEdgeIndex;
   }

   public boolean isObsolete()
   {
      return obsolete;
   }

   public Vector3d getClosestPointToOrigin()
   {
      return closestPointToOrigin;
   }

   public void setObsolete()
   {
      this.obsolete = true;
   }

   public ExpandingPolytopeEntry getAdjacentTriangle(int index)
   {
      return adjacentTriangles[index];
   }

   public int getAdjacentTriangleEdgeIndex(int index)
   {
      return adjacentTriangleEdgeIndices[index];
   }

   private final Vector3d tempVector1 = new Vector3d();
   private final Vector3d tempVector2 = new Vector3d();
   private final Vector3d tempVector3 = new Vector3d();
   private final Vector3d tempVector4 = new Vector3d();
   private final Vector3d tempNormalVector1 = new Vector3d();

   private void projectOriginOntoFace(Point3d vertexOne, Point3d vertexTwo, Point3d vertexThree, Vector3d closestPointToOrigin, double[] lambdas)
   {
      // Using barycentric coordinates as described in https://www.cs.ubc.ca/~heidrich/Papers/JGT.05.pdf
      tempVector1.sub(vertexTwo, vertexOne);
      tempVector2.sub(vertexThree, vertexOne);

      tempNormalVector1.cross(tempVector1, tempVector2);
      double oneOver4ASquared = 1.0 / (tempNormalVector1.dot(tempNormalVector1));

      tempVector3.set(vertexOne);
      tempVector3.scale(-1.0); //w

      tempVector4.cross(tempVector1, tempVector3);
      double lambdaThree = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      tempVector4.cross(tempVector3, tempVector2);
      double lambdaTwo = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      double lambdaOne = 1.0 - lambdaTwo - lambdaThree;

      lambdas[0] = lambdaOne;
      lambdas[1] = lambdaTwo;
      lambdas[2] = lambdaThree;

      closestPointToOrigin.set(0.0, 0.0, 0.0);

      tempVector1.set(vertexOne);
      tempVector1.scale(lambdaOne);
      closestPointToOrigin.add(tempVector1);

      tempVector1.set(vertexTwo);
      tempVector1.scale(lambdaTwo);
      closestPointToOrigin.add(tempVector1);

      tempVector1.set(vertexThree);
      tempVector1.scale(lambdaThree);
      closestPointToOrigin.add(tempVector1);
   }

   @Override
   public int compareTo(ExpandingPolytopeEntry entry)
   {
      if (this.distanceToOriginKey == entry.distanceToOriginKey)
         return 0;
      if (this.distanceToOriginKey > entry.distanceToOriginKey)
         return 1;
      return -1;
   }

   public boolean isAffinelyDependent()
   {
      // TODO: Implement and test this!
      return false;
   }

}
