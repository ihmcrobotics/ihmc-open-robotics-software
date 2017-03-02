package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Data structure for the Expanding Polytope Algorithm described in "Collision Detection in Interactive 3D Environments"
 * by Gino van den Bergen. Vertices and edges are index counterclockwise.
 *
 */
public class ExpandingPolytopeEntry implements Comparable<ExpandingPolytopeEntry>
{
   private final Point3D[] triangleVertices = new Point3D[3];
   private final Vector3D closestPointToOrigin = new Vector3D();
   private final double[] lambdas = new double[3];
   private double distanceToOriginKey;

   private final ExpandingPolytopeEntry[] adjacentTriangles = new ExpandingPolytopeEntry[3];
   private final int[] adjacentTriangleEdgeIndices = new int[3];

   private boolean affinelyDependent = false;
   private boolean obsolete = false;

   public ExpandingPolytopeEntry()
   {
      
   }

   public ExpandingPolytopeEntry(Point3D vertexOne, Point3D vertexTwo, Point3D vertexThree)
   {
      reset(vertexOne, vertexTwo, vertexThree);
   }

   public void reset(Point3D pointOne, Point3D pointTwo, Point3D pointThree)
   {
      this.obsolete = false;
      this.affinelyDependent = false;

      adjacentTriangles[0] = null;
      adjacentTriangles[1] = null;
      adjacentTriangles[2] = null;

      lambdas[0] = 0.0;
      lambdas[1] = 0.0;
      lambdas[2] = 0.0;

      distanceToOriginKey = 0.0;

      adjacentTriangleEdgeIndices[0] = 0;
      adjacentTriangleEdgeIndices[1] = 0;
      adjacentTriangleEdgeIndices[2] = 0;
      
      closestPointToOrigin.set(0.0, 0.0, 0.0);

      triangleVertices[0] = pointOne;
      triangleVertices[1] = pointTwo;
      triangleVertices[2] = pointThree;

      projectOriginOntoFace(pointOne, pointTwo, pointThree, closestPointToOrigin, lambdas);
      distanceToOriginKey = closestPointToOrigin.length();
      if (Double.isNaN(distanceToOriginKey))
      {
         this.affinelyDependent = true;
         //         throw new RuntimeException("Distance to origin is NaN!. \npointOne = " + pointOne + ", \npointTwo = " + pointTwo + ", \npointThree = " + pointThree);
      }
   }

   public Point3D getVertex(int index)
   {
      return triangleVertices[index];
   }

   public double getLambda(int index)
   {
      return lambdas[index];
   }

   public boolean closestIsInternal()
   {
      for (int i = 0; i < 3; i++)
      {
         if (lambdas[i] < 0.0)
            return false;
         if (lambdas[i] > 1.0)
            return false;
      }

      return true;
   }

   /**
    *
    * @param thisTriangleEdgeIndex
    * @param adjacentTriangleEntry
    * @param adjacentTriangleEdgeIndex
    * @return true if the set made a difference, false if it was already set that way.
    */
   public boolean setAdjacentTriangle(int thisTriangleEdgeIndex, ExpandingPolytopeEntry adjacentTriangleEntry, int adjacentTriangleEdgeIndex)
   {
      boolean alreadySetThisWay = false;
      if (adjacentTriangles[thisTriangleEdgeIndex] == adjacentTriangleEntry)
      {
         alreadySetThisWay = true;
      }

      adjacentTriangles[thisTriangleEdgeIndex] = adjacentTriangleEntry;
      adjacentTriangleEdgeIndices[thisTriangleEdgeIndex] = adjacentTriangleEdgeIndex;

      return !alreadySetThisWay;
   }

   public boolean isObsolete()
   {
      return obsolete;
   }

   public Vector3D getClosestPointToOrigin()
   {
      return closestPointToOrigin;
   }

   public void setObsolete()
   {
      this.obsolete = true;
   }

   public void clearObsolete()
   {
      this.obsolete = false;
   }

   public ExpandingPolytopeEntry getAdjacentTriangle(int index)
   {
      return adjacentTriangles[index];
   }

   public int getAdjacentTriangleEdgeIndex(int index)
   {
      return adjacentTriangleEdgeIndices[index];
   }

   private final Vector3D tempVector1 = new Vector3D();
   private final Vector3D tempVector2 = new Vector3D();
   private final Vector3D tempVector3 = new Vector3D();
   private final Vector3D tempVector4 = new Vector3D();
   private final Vector3D tempNormalVector1 = new Vector3D();

   private void projectOriginOntoFace(Point3D vertexOne, Point3D vertexTwo, Point3D vertexThree, Vector3D closestPointToOrigin, double[] lambdas)
   {
      // Using barycentric coordinates as described in https://www.cs.ubc.ca/~heidrich/Papers/JGT.05.pdf
      tempVector1.sub(vertexTwo, vertexOne);
      tempVector2.sub(vertexThree, vertexOne);

      tempNormalVector1.cross(tempVector1, tempVector2);
      double fourASquared = tempNormalVector1.dot(tempNormalVector1);

      //TODO: Magic number for checking affinely Dependent...
      // Probably a better way to check this than just the area of the triangle.
      // Something relative. 
      this.affinelyDependent = fourASquared < 1e-10;

      double oneOver4ASquared = 1.0 / (fourASquared);

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
      return affinelyDependent;
   }

   public boolean setAdjacentTriangleIfPossible(ExpandingPolytopeEntry entry)
   {
      if (this == entry)
         return false;

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            if ((triangleVertices[i] == entry.triangleVertices[j]) && (triangleVertices[(i + 1) % 3] == entry.triangleVertices[(j - 1 + 3) % 3]))
            {
               boolean madeADifference = this.setAdjacentTriangle(i, entry, (j - 1 + 3) % 3);
               entry.setAdjacentTriangle((j - 1 + 3) % 3, this, i);
               return madeADifference;
            }

            if ((triangleVertices[i] == entry.triangleVertices[j]) && (triangleVertices[(i + 1) % 3] == entry.triangleVertices[(j + 1) % 3]))
            {
               entry.swapTrianglesToReverseClockwiseness((j + 1) % 3, (j - 1 + 3) % 3);

               // Now same as case above. Do same thing:
               boolean madeADifference = this.setAdjacentTriangle(i, entry, (j - 1 + 3) % 3);
               entry.setAdjacentTriangle((j - 1 + 3) % 3, this, i);
               return madeADifference;
            }
         }
      }

      return false;
   }

   private void swapTrianglesToReverseClockwiseness(int i, int j)
   {
      Point3D temp = triangleVertices[i];
      triangleVertices[i] = triangleVertices[j];
      triangleVertices[j] = temp;

      if ((adjacentTriangles[0] != null) || (adjacentTriangles[0] != null) || (adjacentTriangles[0] != null))
      {
         throw new RuntimeException("Cannot swap triangles if it already has a neighbor!");
      }
   }

   public boolean isAdjacentTo(ExpandingPolytopeEntry entry)
   {
      if (this.adjacentTriangles[0] == entry)
         return true;
      if (this.adjacentTriangles[1] == entry)
         return true;
      if (this.adjacentTriangles[2] == entry)
         return true;
      return false;
   }

   public void checkConsistency()
   {
      for (int i = 0; i < 3; i++)
      {
         ExpandingPolytopeEntry adjacentTriangle = adjacentTriangles[i];

         if (adjacentTriangle == null)
         {
            throw new RuntimeException("Adjacent triangle is null. Not fully connected!");
         }

         //         if (adjacentTriangle != null)
         //         {
         int j = this.adjacentTriangleEdgeIndices[i];
         if (adjacentTriangle.adjacentTriangles[j] != this)
            throw new RuntimeException("Adjacent triangle was not connected properly!");
         if (adjacentTriangle.adjacentTriangleEdgeIndices[j] != i)
            throw new RuntimeException("Adjacent triangle was not connected properly!");

         Point3D firstVertex = this.triangleVertices[i];
         Point3D secondVertex = this.triangleVertices[(i + 1) % 3];

         Point3D firstVertexOtherSide = adjacentTriangle.triangleVertices[j];
         Point3D secondVertexOtherSide = adjacentTriangle.triangleVertices[(j + 1) % 3];

         if (firstVertex != secondVertexOtherSide)
            throw new RuntimeException("");
         if (secondVertex != firstVertexOtherSide)
            throw new RuntimeException("");
         //         }
      }
   }

   public void getAllConnectedTriangles(ArrayList<ExpandingPolytopeEntry> trianglesToPack)
   {
      if (!trianglesToPack.contains(this))
      {
         trianglesToPack.add(this);

         for (int i = 0; i < 3; i++)
         {
            ExpandingPolytopeEntry adjacentTriangle = this.adjacentTriangles[i];
            if (adjacentTriangle != null)
            {
               adjacentTriangle.getAllConnectedTriangles(trianglesToPack);
            }
         }
      }
   }

   public String toString()
   {
      return "[" + triangleVertices[0] + "; " + triangleVertices[1] + "; " + triangleVertices[2] + "]";
   }

}
