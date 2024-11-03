package us.ihmc.robotics.math;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class DeadbandToolsTest
{
   private static final double epsilon = 1e-9;
   private static final int iters = 100;

   @Test
   public void testScalar()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         double value = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double noPassDeadband = RandomNumbers.nextDouble(random, Math.abs(value) + 10.0 * epsilon, 100);
         double somePassDeadband = RandomNumbers.nextDouble(random, Math.abs(value) - 10.0 * epsilon);

         double offset = RandomNumbers.nextDouble(random, 100.0);

         assertEquals("iter = " + iter, 0.0, DeadbandTools.applyDeadband(noPassDeadband, value), epsilon);
         assertEquals("iter = " + iter, 0.0, DeadbandTools.applyDeadband(noPassDeadband, -value), epsilon);

         assertEquals("iter = " + iter, offset, DeadbandTools.applyDeadband(noPassDeadband, offset, offset + value));
         assertEquals("iter = " + iter, offset, DeadbandTools.applyDeadband(noPassDeadband, offset, offset - value));

         assertEquals("Deadband = " + somePassDeadband + ", value = " + value, value - somePassDeadband, DeadbandTools.applyDeadband(somePassDeadband, value), epsilon);
         assertEquals("Deadband = " + somePassDeadband + ", value = " + (-value), -value + somePassDeadband, DeadbandTools.applyDeadband(somePassDeadband, -value), epsilon);

         assertEquals("Center = " + offset + ", deadband = " + somePassDeadband + ", value = " + (offset + value), offset + value - somePassDeadband, DeadbandTools.applyDeadband(somePassDeadband, offset, offset + value), epsilon);
         assertEquals("Center = " + offset + ", deadband = " + somePassDeadband + ", value = " + (offset - value), offset - value + somePassDeadband, DeadbandTools.applyDeadband(somePassDeadband, offset, offset - value), epsilon);
      }
   }

   @Test
   public void testVector()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         Vector2DReadOnly value2D = EuclidCoreRandomTools.nextVector2D(random, new Vector2D(10.0, 10.0));
         Vector3DReadOnly value3D = EuclidCoreRandomTools.nextVector3D(random, new Vector3D(10.0, 10.0, 10.0));
         double noPassDeadband2D = RandomNumbers.nextDouble(random, value2D.length() + 10.0 * epsilon, 100);
         double noPassDeadband3D = RandomNumbers.nextDouble(random, value3D.length() + 10.0 * epsilon, 100);
         double somePassDeadband2D = RandomNumbers.nextDouble(random, 0.0, value2D.length() - 10.0 * epsilon);
         double somePassDeadband3D = RandomNumbers.nextDouble(random, 0.0, value3D.length() - 10.0 * epsilon);

         Vector2D output2D = new Vector2D();
         Vector3D output3D = new Vector3D();

         output2D.set(value2D);
         output3D.set(value3D);
         DeadbandTools.applyDeadband(output2D, noPassDeadband2D);
         DeadbandTools.applyDeadband(output3D, noPassDeadband3D);
         EuclidCoreTestTools.assertVector2DGeometricallyEquals(new Vector2D(), output2D, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), output3D, epsilon);

         output2D.set(value2D);
         output3D.set(value3D);
         output2D.negate();
         output3D.negate();
         DeadbandTools.applyDeadband(output2D, noPassDeadband2D);
         DeadbandTools.applyDeadband(output3D, noPassDeadband3D);
         EuclidCoreTestTools.assertVector2DGeometricallyEquals(new Vector2D(), output2D, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), output3D, epsilon);


         output2D.set(value2D);
         DeadbandTools.applyDeadband(output2D, somePassDeadband2D);
         Vector2D output2DExpected = new Vector2D(value2D);
         output2DExpected.normalize();
         output2DExpected.scale(-somePassDeadband2D);
         output2DExpected.add(value2D);
         EuclidCoreTestTools.assertVector2DGeometricallyEquals(output2DExpected, output2D, epsilon);


         output3D.set(value3D);
         DeadbandTools.applyDeadband(output3D, somePassDeadband3D);
         Vector3D output3DExpected = new Vector3D(value3D);
         output3DExpected.normalize();
         output3DExpected.scale(-somePassDeadband3D);
         output3DExpected.add(value3D);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(output3DExpected, output3D, epsilon);

      }
   }

   @Test
   public void testPoint()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {

         Point2DReadOnly value2D = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2DReadOnly center2D = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point3DReadOnly value3D = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DReadOnly center3D = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         double noPassDeadband2D = RandomNumbers.nextDouble(random, value2D.distance(center2D) + 1e3 * epsilon, 200);
         double noPassDeadband3D = RandomNumbers.nextDouble(random, value3D.distance(center3D) + 1e3 * epsilon, 200);
         double somePassDeadband2D = RandomNumbers.nextDouble(random, 0.0, value2D.distance(center2D) - 100.0 * epsilon);
         double somePassDeadband3D = RandomNumbers.nextDouble(random, 0.0, value3D.distance(center3D) - 100.0 * epsilon);

         Point2D output2D = new Point2D();
         Point3D output3D = new Point3D();

         output2D.set(value2D);
         output3D.set(value3D);
         DeadbandTools.applyDeadband(output2D, center2D, noPassDeadband2D);
         DeadbandTools.applyDeadband(output3D, center3D, noPassDeadband3D);
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals("iter = " + iter, center2D, output2D, epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("iter = " + iter, center3D, output3D, epsilon);

         output2D.set(value2D);
         DeadbandTools.applyDeadband(output2D, center2D, somePassDeadband2D);
         Vector2D deadband2DVector = new Vector2D(value2D);
         deadband2DVector.sub(center2D);
         deadband2DVector.normalize();
         deadband2DVector.scale(-somePassDeadband2D);
         Point2D output2DExpected = new Point2D(value2D);
         output2DExpected.add(deadband2DVector);
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals("iter = " + iter, output2DExpected, output2D, epsilon);


         output3D.set(value3D);
         DeadbandTools.applyDeadband(output3D, center3D, somePassDeadband3D);
         Vector3D deadband3DVector = new Vector3D(value3D);
         deadband3DVector.sub(center3D);
         deadband3DVector.normalize();
         deadband3DVector.scale(-somePassDeadband3D);
         Point3D output3DExpected = new Point3D(value3D);
         output3DExpected.add(deadband3DVector);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("iter = " + iter, output3DExpected, output3D, epsilon);
      }
   }
}
