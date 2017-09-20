package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

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
   public void computedSupportingVertices(Point3D supportingVertexA, Point3D supportingVertexB, Vector3D w)
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
   public void foundMinimumPenetrationVector(Vector3D minimumPenetrationVector, Point3D closestPointOnA, Point3D closestPointOnB)
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
