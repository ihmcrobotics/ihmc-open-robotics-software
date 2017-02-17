package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Point3D;

public class ExpandingPolytopeEntryFromSimpleMeshGeneratorTest
{

   @Test
   public void testExpandingPolytopeEntryFromSimpleMeshGeneratorWithTetrahedronCorrectlyOrdered()
   {
      SimpleTriangleMesh tetrahedron = new SimpleTriangleMesh();

      tetrahedron.positions.add(new Point3D(0.0, 0.0, 0.0));
      tetrahedron.positions.add(new Point3D(0.0, 2.0, 0.0));
      tetrahedron.positions.add(new Point3D(1.0, 2.0, 0.0));
      tetrahedron.positions.add(new Point3D(1.0, 1.0, 1.0));

      tetrahedron.triangleIndices.add(0);
      tetrahedron.triangleIndices.add(1);
      tetrahedron.triangleIndices.add(2);

      tetrahedron.triangleIndices.add(1);
      tetrahedron.triangleIndices.add(3);
      tetrahedron.triangleIndices.add(2);

      tetrahedron.triangleIndices.add(0);
      tetrahedron.triangleIndices.add(3);
      tetrahedron.triangleIndices.add(1);

      tetrahedron.triangleIndices.add(0);
      tetrahedron.triangleIndices.add(2);
      tetrahedron.triangleIndices.add(3);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytope = generator.generateExpandingPolytope(tetrahedron);

      ArrayList<ExpandingPolytopeEntry> allTriangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(allTriangles);

      assertEquals(tetrahedron.triangleIndices.size()/3, allTriangles.size());

      for (ExpandingPolytopeEntry triangle : allTriangles)
      {
         triangle.checkConsistency();
      }
   }

   @Test
   public void testExpandingPolytopeEntryFromSimpleMeshGeneratorWithTetrahedronIncorrectlyOrdered()
   {
      SimpleTriangleMesh tetrahedron = new SimpleTriangleMesh();

      tetrahedron.positions.add(new Point3D(0.0, 0.0, 0.0));
      tetrahedron.positions.add(new Point3D(0.0, 2.0, 0.0));
      tetrahedron.positions.add(new Point3D(1.0, 2.0, 0.0));
      tetrahedron.positions.add(new Point3D(1.0, 1.0, 1.0));

      tetrahedron.triangleIndices.add(0);
      tetrahedron.triangleIndices.add(1);
      tetrahedron.triangleIndices.add(2);

      // This triangle is not pointing the same way as the other three.
      tetrahedron.triangleIndices.add(1);
      tetrahedron.triangleIndices.add(2);
      tetrahedron.triangleIndices.add(3);

      tetrahedron.triangleIndices.add(0);
      tetrahedron.triangleIndices.add(3);
      tetrahedron.triangleIndices.add(1);

      tetrahedron.triangleIndices.add(0);
      tetrahedron.triangleIndices.add(2);
      tetrahedron.triangleIndices.add(3);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytope = generator.generateExpandingPolytope(tetrahedron);

      ArrayList<ExpandingPolytopeEntry> allTriangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(allTriangles);

      assertEquals(tetrahedron.triangleIndices.size()/3, allTriangles.size());

      for (ExpandingPolytopeEntry triangle : allTriangles)
      {
         triangle.checkConsistency();
      }
   }



   @Test
   public void testExpandingPolytopeEntryFromSimpleMeshGeneratorWithIcoSpheres()
   {
      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      IcoSphereCreator creator = new IcoSphereCreator();

      SimpleTriangleMesh icoSphere = creator.createIcoSphere(0);

      ExpandingPolytopeEntry expandingPolytope = generator.generateExpandingPolytope(icoSphere);

      ArrayList<ExpandingPolytopeEntry> allTriangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(allTriangles);

      assertEquals(icoSphere.triangleIndices.size()/3, allTriangles.size());

      for (ExpandingPolytopeEntry triangle : allTriangles)
      {
         triangle.checkConsistency();
      }

   }

}
