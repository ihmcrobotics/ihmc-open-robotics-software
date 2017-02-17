package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PrintingExpandingPolytopeAlgorithmListener implements ExpandingPolytopeAlgorithmListener
{

   @Override
   public void setPolytopes(SimplexPolytope simplex, SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB, ExpandingPolytopeEntry triangleEntry)
   {
      System.out.println("\n-----------\nExpandingPolytopeAlgorithm -- Setting Polytopes:");
      System.out.println("simplex:" + simplex);
      System.out.println("polytopeA:" + polytopeA);
      System.out.println("polytopeB:" + polytopeB);
   }

   @Override
   public void polledEntryToExpand(ExpandingPolytopeEntry triangleEntryToExpand)
   {
      System.out.println("Polled entry to expand:\n" + triangleEntryToExpand);
      System.out.println("is obsolete = " + triangleEntryToExpand.isObsolete());
   }

   @Override
   public void computedSupportingVertices(Point3D supportingVertexA, Point3D supportingVertexB, Vector3D w)
   {
      System.out.println("Computed Supporting Vertex:" + w);
   }

   @Override
   public void computedCloseEnough(double vDotW, double lengthSquared, double mu, boolean closeEnough)
   {
      System.out.println("Computed close enough:" + closeEnough);
   }

   @Override
   public void computedSilhouetteFromW(ExpandingPolytopeEdgeList edgeList)
   {
      System.out.println("computed Silhouette From W with " + edgeList.getNumberOfEdges() + " edges.");
   }

   @Override
   public void foundMinimumPenetrationVector(Vector3D minimumPenetrationVector, Point3D closestPointOnA, Point3D closestPointOnB)
   {
      System.out.println("Found Minimum Penetration Vector:" + minimumPenetrationVector);
      System.out.println("closestPointOnA = " + closestPointOnA);
      System.out.println("closestPointOnB = " + closestPointOnB);
   }

   @Override
   public void createdNewEntry(ExpandingPolytopeEntry newEntry)
   {
      System.out.println("Created New Entry:" + newEntry);
   }

   @Override
   public void addedNewEntryToQueue(ExpandingPolytopeEntry newEntry)
   {
      System.out.println("Added New Entry To Queue:" + newEntry);
   }

   @Override
   public void expandedPolytope(ExpandingPolytopeEntry firstNewEntry)
   {
      System.out.println("Expanded the polytope. First new entry = " + firstNewEntry);
   }

}
