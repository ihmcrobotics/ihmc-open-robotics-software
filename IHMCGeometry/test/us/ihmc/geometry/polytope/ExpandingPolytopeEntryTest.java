package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Point3D;

public class ExpandingPolytopeEntryTest
{

   @Test
   public void testManuallyAssembledTetrahedron()
   {
      Point3D pointOne = new Point3D(0.0, 0.0, 0.0);
      Point3D pointTwo = new Point3D(2.0, 0.0, 0.0);
      Point3D pointThree = new Point3D(1.0, 2.0, 0.0);
      Point3D pointFour = new Point3D(1.0, 1.0, 1.0);

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

      assertFalse(entry123.isAdjacentTo(entry123));
      assertTrue(entry123.isAdjacentTo(entry324));
      assertTrue(entry123.isAdjacentTo(entry421));
      assertTrue(entry123.isAdjacentTo(entry134));

      assertTrue(entry324.isAdjacentTo(entry123));
      assertFalse(entry324.isAdjacentTo(entry324));
      assertTrue(entry324.isAdjacentTo(entry421));
      assertTrue(entry324.isAdjacentTo(entry134));

      assertTrue(entry421.isAdjacentTo(entry123));
      assertTrue(entry421.isAdjacentTo(entry324));
      assertFalse(entry421.isAdjacentTo(entry421));
      assertTrue(entry421.isAdjacentTo(entry134));

      assertTrue(entry134.isAdjacentTo(entry123));
      assertTrue(entry134.isAdjacentTo(entry324));
      assertTrue(entry134.isAdjacentTo(entry421));
      assertFalse(entry134.isAdjacentTo(entry134));

      entry123.checkConsistency();
      entry324.checkConsistency();
      entry421.checkConsistency();
      entry134.checkConsistency();


   }

   @Test
   public void testAutomaticallyAssembledTetrahedron()
   {
      Point3D pointOne = new Point3D(0.0, 0.0, 0.0);
      Point3D pointTwo = new Point3D(2.0, 0.0, 0.0);
      Point3D pointThree = new Point3D(1.0, 2.0, 0.0);
      Point3D pointFour = new Point3D(1.0, 1.0, 1.0);

      ExpandingPolytopeEntry entry123 = new ExpandingPolytopeEntry(pointOne, pointTwo, pointThree);
      ExpandingPolytopeEntry entry324 = new ExpandingPolytopeEntry(pointThree, pointTwo, pointFour);
      ExpandingPolytopeEntry entry421 = new ExpandingPolytopeEntry(pointFour, pointTwo, pointOne);
      ExpandingPolytopeEntry entry134 = new ExpandingPolytopeEntry(pointOne, pointThree, pointFour);

      entry123.setAdjacentTriangleIfPossible(entry123);
      entry123.setAdjacentTriangleIfPossible(entry324);
      entry123.setAdjacentTriangleIfPossible(entry421);
      entry123.setAdjacentTriangleIfPossible(entry134);

      entry324.setAdjacentTriangleIfPossible(entry123);
      entry324.setAdjacentTriangleIfPossible(entry324);
      entry324.setAdjacentTriangleIfPossible(entry421);
      entry324.setAdjacentTriangleIfPossible(entry134);

      entry421.setAdjacentTriangleIfPossible(entry123);
      entry421.setAdjacentTriangleIfPossible(entry324);
      entry421.setAdjacentTriangleIfPossible(entry421);
      entry421.setAdjacentTriangleIfPossible(entry134);

      entry134.setAdjacentTriangleIfPossible(entry123);
      entry134.setAdjacentTriangleIfPossible(entry324);
      entry134.setAdjacentTriangleIfPossible(entry421);
      entry134.setAdjacentTriangleIfPossible(entry134);

      assertFalse(entry123.isAdjacentTo(entry123));
      assertTrue(entry123.isAdjacentTo(entry324));
      assertTrue(entry123.isAdjacentTo(entry421));
      assertTrue(entry123.isAdjacentTo(entry134));

      assertTrue(entry324.isAdjacentTo(entry123));
      assertFalse(entry324.isAdjacentTo(entry324));
      assertTrue(entry324.isAdjacentTo(entry421));
      assertTrue(entry324.isAdjacentTo(entry134));

      assertTrue(entry421.isAdjacentTo(entry123));
      assertTrue(entry421.isAdjacentTo(entry324));
      assertFalse(entry421.isAdjacentTo(entry421));
      assertTrue(entry421.isAdjacentTo(entry134));

      assertTrue(entry134.isAdjacentTo(entry123));
      assertTrue(entry134.isAdjacentTo(entry324));
      assertTrue(entry134.isAdjacentTo(entry421));
      assertFalse(entry134.isAdjacentTo(entry134));

      entry123.checkConsistency();
      entry324.checkConsistency();
      entry421.checkConsistency();
      entry134.checkConsistency();


   }
   
   
   @Test
   public void testTroublesomeTetrahedron()
   {         
      Point3D pointOne = new Point3D(-0.9490913237594096, 0.30106983928716424, 3.560641277995469);
      Point3D pointTwo = new Point3D(-0.9895570462032808, -0.7156529059809014, -1.2414394231490622);
      Point3D pointThree = new Point3D(1.3112536912820332, 1.1824049227088569, 1.3675055136001983);
      Point3D pointFour = new Point3D(-0.799289825570983, -2.2676996322425795, 0.9981915411767837);

      ExpandingPolytopeEntry entry123 = new ExpandingPolytopeEntry(pointOne, pointTwo, pointThree);
      ExpandingPolytopeEntry entry324 = new ExpandingPolytopeEntry(pointThree, pointTwo, pointFour);
      ExpandingPolytopeEntry entry421 = new ExpandingPolytopeEntry(pointFour, pointTwo, pointOne);
      ExpandingPolytopeEntry entry134 = new ExpandingPolytopeEntry(pointOne, pointThree, pointFour);

      entry123.setAdjacentTriangleIfPossible(entry123);
      entry123.setAdjacentTriangleIfPossible(entry324);
      entry123.setAdjacentTriangleIfPossible(entry421);
      entry123.setAdjacentTriangleIfPossible(entry134);

      entry324.setAdjacentTriangleIfPossible(entry123);
      entry324.setAdjacentTriangleIfPossible(entry324);
      entry324.setAdjacentTriangleIfPossible(entry421);
      entry324.setAdjacentTriangleIfPossible(entry134);

      entry421.setAdjacentTriangleIfPossible(entry123);
      entry421.setAdjacentTriangleIfPossible(entry324);
      entry421.setAdjacentTriangleIfPossible(entry421);
      entry421.setAdjacentTriangleIfPossible(entry134);

      entry134.setAdjacentTriangleIfPossible(entry123);
      entry134.setAdjacentTriangleIfPossible(entry324);
      entry134.setAdjacentTriangleIfPossible(entry421);
      entry134.setAdjacentTriangleIfPossible(entry134);

      assertFalse(entry123.isAdjacentTo(entry123));
      assertTrue(entry123.isAdjacentTo(entry324));
      assertTrue(entry123.isAdjacentTo(entry421));
      assertTrue(entry123.isAdjacentTo(entry134));

      assertTrue(entry324.isAdjacentTo(entry123));
      assertFalse(entry324.isAdjacentTo(entry324));
      assertTrue(entry324.isAdjacentTo(entry421));
      assertTrue(entry324.isAdjacentTo(entry134));

      assertTrue(entry421.isAdjacentTo(entry123));
      assertTrue(entry421.isAdjacentTo(entry324));
      assertFalse(entry421.isAdjacentTo(entry421));
      assertTrue(entry421.isAdjacentTo(entry134));

      assertTrue(entry134.isAdjacentTo(entry123));
      assertTrue(entry134.isAdjacentTo(entry324));
      assertTrue(entry134.isAdjacentTo(entry421));
      assertFalse(entry134.isAdjacentTo(entry134));

      entry123.checkConsistency();
      entry324.checkConsistency();
      entry421.checkConsistency();
      entry134.checkConsistency();


   }

}
