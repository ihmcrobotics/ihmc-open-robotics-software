package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

public class ExpandingPolytopeSilhouetteConstructorTest
{

   @Test
   public void testWithTetrahedron()
   {
      Point3d pointOne = new Point3d(-1.0, -1.0, -1.0);
      Point3d pointTwo = new Point3d(1.0, -1.0, -1.0);
      Point3d pointThree = new Point3d(0.0, 1.0, -1.0);
      Point3d pointFour = new Point3d(0.0, 0.0, 1.0);

      ExpandingPolytopeEntry entry123 = new ExpandingPolytopeEntry(pointOne, pointTwo, pointThree);
      ExpandingPolytopeEntry entry324 = new ExpandingPolytopeEntry(pointThree, pointTwo, pointFour);
      ExpandingPolytopeEntry entry421 = new ExpandingPolytopeEntry(pointFour, pointTwo, pointOne);
      ExpandingPolytopeEntry entry134 = new ExpandingPolytopeEntry(pointOne, pointThree, pointFour);

      entry123.setAdjacentTriangle(1, entry324, 0);
      entry324.setAdjacentTriangle(0, entry123, 1);

      entry123.setAdjacentTriangle(0, entry421, 1);
      entry421.setAdjacentTriangle(1, entry123, 0);

      entry123.setAdjacentTriangle(2, entry134, 0);
      entry134.setAdjacentTriangle(0, entry123, 2);

      entry324.setAdjacentTriangle(1, entry421, 0);
      entry421.setAdjacentTriangle(0, entry324, 1);

      entry324.setAdjacentTriangle(2, entry134, 1);
      entry134.setAdjacentTriangle(1, entry324, 2);

      entry421.setAdjacentTriangle(2, entry134, 2);
      entry134.setAdjacentTriangle(2, entry421, 2);

      Vector3d w = new Vector3d(0.0, 0.0, 1.5);
      ExpandingPolytopeEdgeList edgeListToPack = new ExpandingPolytopeEdgeList();
      ExpandingPolytopeSilhouetteConstructor.computeSilhouetteFromW(entry421, w, edgeListToPack);

      int numberOfEdges = edgeListToPack.getNumberOfEdges();
      assertEquals(3, numberOfEdges);

      ExpandingPolytopeEdge edge = edgeListToPack.getEdge(0);
      assertTrue(edge.getEntry() == entry123);
      assertEquals(2, edge.getEdgeIndex());

      edge = edgeListToPack.getEdge(1);
      assertTrue(edge.getEntry() == entry123);
      assertEquals(1, edge.getEdgeIndex());

      edge = edgeListToPack.getEdge(2);
      assertTrue(edge.getEntry() == entry123);
      assertEquals(0, edge.getEdgeIndex());
   }


   @Test
   public void testWithIcoSphere()
   {
      IcoSphereCreator creator = new IcoSphereCreator();
      int recursionLevel = 0;
      SimpleTriangleMesh icoSphere = creator.createIcoSphere(recursionLevel);

      Vector3d w = new Vector3d(0.0, 0.0, 1.5);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytope = generator.generateExpandingPolytope(icoSphere);

      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(triangles);

      for (ExpandingPolytopeEntry triangle : triangles)
      {
         clearObsolete(triangles);

         if (ExpandingPolytopeSilhouetteConstructor.isSeeableFromW(triangle.getClosestPointToOrigin(), w))
         {
            ExpandingPolytopeEdgeList edgeListToPack = new ExpandingPolytopeEdgeList();
            ExpandingPolytopeSilhouetteConstructor.computeSilhouetteFromW(expandingPolytope, w, edgeListToPack);

            int numberOfEdges = edgeListToPack.getNumberOfEdges();
            assertEquals(8, numberOfEdges);

            System.out.print("\n\nEdgeList:\n");

            for (int i=0; i<numberOfEdges; i++)
            {
               ExpandingPolytopeEdge edge = edgeListToPack.getEdge(i);
               ExpandingPolytopeEntry entry = edge.getEntry();
               int edgeIndex = edge.getEdgeIndex();
               Point3d vertexOne = entry.getVertex(edgeIndex);
               Point3d vertexTwo = entry.getVertex((edgeIndex + 1) %3);
               System.out.println(vertexOne + " -- " + vertexTwo);
            }
         }
      }
   }


   private void clearObsolete(ArrayList<ExpandingPolytopeEntry> triangles)
   {
      for (ExpandingPolytopeEntry triangle : triangles)
      {
         triangle.clearObsolete();
      }

   }

}
