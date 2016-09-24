package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import org.junit.Test;

public class ConvexPolytopeFromExpandingPolytopeEntryGeneratorTest
{

   @Test
   public void test()
   {
      IcoSphereCreator creator = new IcoSphereCreator();
      int recursionLevel = 0;
      SimpleTriangleMesh icoSphere = creator.createIcoSphere(recursionLevel);

      ArrayList<Point3d> vertexPoints = icoSphere.positions;
      ArrayList<Integer> triangleIndices = icoSphere.triangleIndices;
      assertTrue(triangleIndices.size() % 3 == 0);

      int numberOfVertices = vertexPoints.size();
      int numberOfTriangles = triangleIndices.size() / 3;

      assertEquals(12, numberOfVertices);
      assertEquals(20, numberOfTriangles);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generatorOne = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytope = generatorOne.generateExpandingPolytope(icoSphere);

      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(triangles);
      assertEquals(numberOfTriangles, triangles.size());

      ConvexPolytopeFromExpandingPolytopeEntryGenerator generatorTwo = new ConvexPolytopeFromExpandingPolytopeEntryGenerator();
      ConvexPolytope convexPolytope = generatorTwo.generateConvexPolytope(expandingPolytope);

      ArrayList<PolytopeVertex[]> edges = convexPolytope.getEdges();
      ArrayList<PolytopeVertex> vertices = convexPolytope.getVertices();

      assertEquals(numberOfVertices, vertices.size());
   }

}
