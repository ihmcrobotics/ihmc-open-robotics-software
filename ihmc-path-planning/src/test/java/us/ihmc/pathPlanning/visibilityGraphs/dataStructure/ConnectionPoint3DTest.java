package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class ConnectionPoint3DTest
{
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-12;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAndGetRounded()
   {
      ConnectionPoint3D point = new ConnectionPoint3D(1.00002, 2.00014, 3.00026, 144);

      assertEquals(144, point.getRegionId());

      assertEquals(1.00002, point.getX(), EPSILON);
      assertEquals(1.0, ConnectionPoint3D.round(point.getX()), EPSILON);

      assertEquals(2.00014, point.getY(), EPSILON);
      assertEquals(2.0001, ConnectionPoint3D.round(point.getY()), EPSILON);

      assertEquals(3.00026, point.getZ(), EPSILON);
      assertEquals(3.0003, ConnectionPoint3D.round(point.getZ()), EPSILON);

      assertEquals(-1749098234, point.hashCode());

      Tuple2DReadOnly tuple = new Point2D(1.7000055, -3.40003);
      point = new ConnectionPoint3D(tuple, 149);

      assertEquals(149, point.getRegionId());

      assertEquals(1.7000055, point.getX(), EPSILON);
      assertEquals(1.7, ConnectionPoint3D.round(point.getX()), EPSILON);

      assertEquals(-3.40003, point.getY(), EPSILON);
      assertEquals(-3.4, ConnectionPoint3D.round(point.getY()), EPSILON);

      assertEquals(0.0, point.getZ(), EPSILON);
      assertEquals(0.0, ConnectionPoint3D.round(point.getZ()), EPSILON);

      assertEquals(154188729, point.hashCode());

      ConnectionPoint3D pointCopy = new ConnectionPoint3D(point);
      assertEquals(pointCopy, point);
      assertEquals(point.getX(), pointCopy.getX(), EPSILON);
      assertEquals(point.getY(), pointCopy.getY(), EPSILON);
      assertEquals(point.getZ(), pointCopy.getZ(), EPSILON);
      assertEquals(point.getRegionId(), pointCopy.getRegionId());

      String outputString = point.toString();
      assertEquals("ConnectionPoint3D: ( 1.700, -3.400,  0.000 )", outputString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testToString()
   {
      Tuple2DReadOnly tuple = new Point2D(1.7000055, -3.40003);
      ConnectionPoint3D point = new ConnectionPoint3D(tuple, 149);

      String outputString = point.toString();
      assertEquals("ConnectionPoint3D: ( 1.700, -3.400,  0.000 )", outputString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHashCode()
   {
      ConnectionPoint3D point = new ConnectionPoint3D(1.00002, 2.00014, 3.00026, 144);

      int hashCode = point.hashCode();
      assertEquals(-1749098234, hashCode);

      ConnectionPoint3D pointTwo = new ConnectionPoint3D(ConnectionPoint3D.round(point.getX()), ConnectionPoint3D.round(point.getY()),
                                                         ConnectionPoint3D.round(point.getZ()), point.getRegionId() + 100);
      assertEquals(hashCode, pointTwo.hashCode());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEquals()
   {
      ConnectionPoint3D pointOne = new ConnectionPoint3D(1.00002, 2.00014, 3.00026, 144);
      ConnectionPoint3D pointTwo = new ConnectionPoint3D(1.0000, 2.0001, 3.0003, 149);
      ConnectionPoint3D pointThree = new ConnectionPoint3D(1.0000, 2.0001, 3.0002, 144);

      assertTrue(pointOne.equals(pointTwo));
      assertTrue(pointTwo.equals(pointOne));
      assertEquals(pointOne.hashCode(), pointTwo.hashCode());

      assertNotEquals(pointOne.hashCode(), pointThree.hashCode());
      assertNotEquals(pointTwo.hashCode(), pointThree.hashCode());

      assertFalse(pointOne.equals(pointThree));
      assertFalse(pointTwo.equals(pointThree));

      ConnectionPoint3D nullConnectionPoint3D = null;

      assertFalse(pointOne.equals(nullConnectionPoint3D));
      assertFalse(pointTwo.equals(nullConnectionPoint3D));
      assertFalse(pointThree.equals(nullConnectionPoint3D));

      Object nullObject = null;

      assertFalse(pointOne.equals(nullObject));
      assertFalse(pointTwo.equals(nullObject));
      assertFalse(pointThree.equals(nullObject));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEqualsTwo()
   {
      // Points one and two are equal as ConnectionPoints, but not as Point3Ds.
      Object pointOne = new ConnectionPoint3D(1.00002, 2.00014, 3.00026, 144);
      Object pointTwo = new ConnectionPoint3D(1.0, 2.0001, 3.0003, 190);

      Object pointOneAsPoint3D = new Point3D(1.00002, 2.00014, 3.00026);
      Object pointTwoAsPoint3D = new Point3D(1.0, 2.0001, 3.0003);

      assertEquals(pointOne, pointTwo);
      assertEquals(pointOne.hashCode(), pointTwo.hashCode());

      assertEquals(pointOne, pointOneAsPoint3D);
      assertEquals(pointOneAsPoint3D, pointOne);

      assertEquals(pointTwo, pointTwoAsPoint3D);
      assertEquals(pointTwoAsPoint3D, pointTwo);

      assertNotEquals(pointOne, pointTwoAsPoint3D);
      assertNotEquals(pointTwoAsPoint3D, pointOne);

      assertNotEquals(pointTwo, pointOneAsPoint3D);
      assertNotEquals(pointOneAsPoint3D, pointTwo);

      assertNotEquals(pointOneAsPoint3D, pointTwoAsPoint3D);

      Object pointThree = new ConnectionPoint3D(1.0000, 2.0001, 3.0002, 144);

      assertNotEquals(pointOne, pointThree);
      assertNotEquals(pointThree, pointOne);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEqualsAndHashConsistency()
   {
      // If two objects are equal, then their hash codes must be equal...
      ConnectionPoint3D pointOne = new ConnectionPoint3D(1.00002, 2.00014, 3.00026, 144);

      for (double delta = -0.1; delta < 0.1; delta = delta + 0.0000001)
      {
         ConnectionPoint3D pointTwo = new ConnectionPoint3D(pointOne.getX() + delta, pointOne.getY(), pointOne.getZ(), pointOne.getRegionId() + 33);

         boolean areEqual = pointOne.equals(pointTwo);
         boolean hashAreEqual = pointOne.hashCode() == pointTwo.hashCode();

         if (areEqual)
         {
            assertTrue(hashAreEqual);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testApplyTransformAndInverseTransform()
   {
      ConnectionPoint3D pointOne = new ConnectionPoint3D(1.0000023, 2.0000028, 3.0000063, 144);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(4.0, -2.0, 9.0);

      ConnectionPoint3D pointTwo = pointOne.applyTransform(transform);

      assertEquals(5.0000023, pointTwo.getX(), EPSILON);
      assertEquals(0.0000028, pointTwo.getY(), EPSILON);
      assertEquals(12.0000063, pointTwo.getZ(), EPSILON);
      assertEquals(144, pointTwo.getRegionId());

      ConnectionPoint3D pointThree = pointTwo.applyInverseTransform(transform);

      assertEquals(1.0000023, pointThree.getX(), EPSILON);
      assertEquals(2.0000028, pointThree.getY(), EPSILON);
      assertEquals(3.0000063, pointThree.getZ(), EPSILON);
      assertEquals(144, pointThree.getRegionId());

      assertTrue(pointThree.equals(pointOne));
      assertEquals(pointThree, pointOne);
      assertEquals(pointThree.getRegionId(), pointOne.getRegionId());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRound() throws Exception
   {
      Random random = new Random(43566787);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double raw = EuclidCoreRandomTools.nextDouble(random, 1000.0);
         double expected = Precision.round(raw, 4);
         double actual = ConnectionPoint3D.round(raw);
         assertEquals(expected, actual, EPSILON);
      }
   }

   public static void main(String[] args) throws IOException
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ConnectionPoint3D.class, ConnectionPoint3DTest.class);
   }
}
