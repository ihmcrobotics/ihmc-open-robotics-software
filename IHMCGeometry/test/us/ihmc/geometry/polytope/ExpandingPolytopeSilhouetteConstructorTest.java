package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;

public class ExpandingPolytopeSilhouetteConstructorTest
{

   @Test
   public void testSilhouetteConstructorWithTetrahedron()
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
   public void testSilhouetteConstructorWithIcoSphere()
   {
      IcoSphereCreator creator = new IcoSphereCreator();
      int recursionLevel = 0;
      SimpleTriangleMesh icoSphere = creator.createIcoSphere(recursionLevel);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytope = generator.generateExpandingPolytope(icoSphere);

      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(triangles);

      // These were all verified by eye using ExpandingPolytopeSilhouetteConstructorVisualizer
      Vector3d w = new Vector3d(0.0, 0.0, 1.37);
      int expectedNumberOfEdgesOnSilhouette = 4;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 1.38);
      expectedNumberOfEdgesOnSilhouette = 8;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 2.22);
      expectedNumberOfEdgesOnSilhouette = 8;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 2.23);
      expectedNumberOfEdgesOnSilhouette = 6;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(1.0, 0.0, 1.23);
      expectedNumberOfEdgesOnSilhouette = 5;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(1.0, 0.0, 1.24);
      expectedNumberOfEdgesOnSilhouette = 6;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(1.0, 0.0, 2.22);
      expectedNumberOfEdgesOnSilhouette = 6;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(1.0, 0.0, 2.23);
      expectedNumberOfEdgesOnSilhouette = 8;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      recursionLevel = 1;
      icoSphere = creator.createIcoSphere(recursionLevel);

      expandingPolytope = generator.generateExpandingPolytope(icoSphere);

      triangles.clear();
      expandingPolytope.getAllConnectedTriangles(triangles);

      // These were all verified by eye using ExpandingPolytopeSilhouetteConstructorVisualizer
      w = new Vector3d(0.0, 0.0, 1.21);
      expectedNumberOfEdgesOnSilhouette = 6;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 1.22);
      expectedNumberOfEdgesOnSilhouette = 8;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 1.26);
      expectedNumberOfEdgesOnSilhouette = 8;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 1.27);
      expectedNumberOfEdgesOnSilhouette = 12;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 1.5);
      expectedNumberOfEdgesOnSilhouette = 12;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 1.51);
      expectedNumberOfEdgesOnSilhouette = 10;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 2.61);
      expectedNumberOfEdgesOnSilhouette = 10;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 2.62);
      expectedNumberOfEdgesOnSilhouette = 12;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 2.99);
      expectedNumberOfEdgesOnSilhouette = 12;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 3.0);
      expectedNumberOfEdgesOnSilhouette = 16;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 4.84);
      expectedNumberOfEdgesOnSilhouette = 16;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);

      w = new Vector3d(0.0, 0.0, 4.85);
      expectedNumberOfEdgesOnSilhouette = 12;
      confirmNumberOfEdgesInSilhouette(triangles, w, expectedNumberOfEdgesOnSilhouette);
   }

   private void confirmNumberOfEdgesInSilhouette(ArrayList<ExpandingPolytopeEntry> triangles, Vector3d w, int expectedNumberOfEdgesOnSilhouette)
   {
      for (ExpandingPolytopeEntry triangle : triangles)
      {
         if (!ExpandingPolytopeSilhouetteConstructor.isNotVisibleFromW(triangle.getClosestPointToOrigin(), w))
         {
            clearObsolete(triangles);

            ExpandingPolytopeEdgeList edgeListToPack = new ExpandingPolytopeEdgeList();
            ExpandingPolytopeSilhouetteConstructor.computeSilhouetteFromW(triangle, w, edgeListToPack);

            int numberOfEdges = edgeListToPack.getNumberOfEdges();
            assertEquals(expectedNumberOfEdgesOnSilhouette, numberOfEdges);
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

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ExpandingPolytopeSilhouetteConstructor.class, ExpandingPolytopeSilhouetteConstructorTest.class);
   }

}
