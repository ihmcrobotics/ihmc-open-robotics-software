package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertFalse;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class ExpandingPolytopeAlgorithmAssertListener implements ExpandingPolytopeAlgorithmListener
{

   @Override
   public void setPolytopes(SimplexPolytope simplex, SupportingVertexHolder polytopeOne, SupportingVertexHolder polytopeTwo, ExpandingPolytopeEntry triangleEntry)
   {      
   }

   @Override
   public void polledEntryToExpand(ExpandingPolytopeEntry triangleEntryToExpand)
   {      
   }

   @Override
   public void computedSupportingVertices(Point3d supportingVertexA, Point3d supportingVertexB, Vector3d w)
   {      
   }

   @Override
   public void computedCloseEnough(double vDotW, double lengthSquared, double mu, boolean closeEnough)
   {      
   }

   @Override
   public void computedSilhouetteFromW(ExpandingPolytopeEdgeList edgeList)
   {      
   }

   @Override
   public void foundMinimumPenetrationVector(Vector3d minimumPenetrationVector, Point3d closestPointOnA, Point3d closestPointOnB)
   {      
   }

   @Override
   public void addedNewEntryToQueue(ExpandingPolytopeEntry newEntry)
   {      
   }

   @Override
   public void createdNewEntry(ExpandingPolytopeEntry newEntry)
   {      
   }

   @Override
   public void expandedPolytope(ExpandingPolytopeEntry firstNewEntry)
   {
      assertFalse(firstNewEntry.isObsolete());
      
      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      firstNewEntry.getAllConnectedTriangles(triangles);
      
      for (int i=0; i<triangles.size(); i++)
      {
         ExpandingPolytopeEntry triangle = triangles.get(i);
         
         assertFalse(triangle.isObsolete());
         triangle.checkConsistency();
      }
   }

}
