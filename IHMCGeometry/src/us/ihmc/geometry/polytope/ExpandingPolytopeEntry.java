package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;

/**
 * Data structure for the Expanding Polytope Algorithm described in "Collision Detection in Interactive 3D Environments"
 * by Gino van den Bergen. Vertices and edges are index counterclockwise.
 *
 */
public class ExpandingPolytopeEntry
{
   private final Point3d[] triangleVertices = new Point3d[3];
   private final Point3d closestPointToOrigin = new Point3d();
   private final double[] lambdas = new double[3];
   private double distanceToOriginKey;

   private final ExpandingPolytopeEntry[] adjacentTriangles = new ExpandingPolytopeEntry[3];
   private final int[] adjacentTriangleEdgeIndex = new int[3];

   private boolean obsolete = false;

   public ExpandingPolytopeEntry()
   {

   }
}
