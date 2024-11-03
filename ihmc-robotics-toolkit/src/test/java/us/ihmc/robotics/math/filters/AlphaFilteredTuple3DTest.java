package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Tuple3DBasicsTest;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class AlphaFilteredTuple3DTest extends Tuple3DBasicsTest<AlphaFilteredTuple3D>
{
   @Test
   public void testFirstSet()
   {
      AlphaFilteredTuple3D tuple = new AlphaFilteredTuple3D(0.0);
      tuple.set(0.0, 1.0, 2.0);

      assertEquals(0.0, tuple.getX(), getEpsilon());
      assertEquals(1.0, tuple.getY(), getEpsilon());
      assertEquals(2.0, tuple.getZ(), getEpsilon());
   }

   @Test
   public void testFilteredSetters()
   {
      Random random = new Random(12951L);
      for (int i = 0; i < 1000; ++i)
      {
         double alpha = random.nextDouble(0.0, 1.0);
         AlphaFilteredTuple3D tuple = createRandomTuple(random);
         tuple.setAlpha(alpha);

         double originalX = tuple.getX();
         double originalY = tuple.getY();
         double originalZ = tuple.getZ();

         Point3D point = new Point3D(createRandomTuple(random));

         tuple.setX(point.getX());
         tuple.setY(point.getY());
         tuple.setZ(point.getZ());

         double expectedX = (1.0 - alpha) * point.getX() + alpha * originalX;
         double expectedY = (1.0 - alpha) * point.getY() + alpha * originalY;
         double expectedZ = (1.0 - alpha) * point.getZ() + alpha * originalZ;

         assertEquals(expectedX, tuple.getX(), getEpsilon());
         assertEquals(expectedY, tuple.getY(), getEpsilon());
         assertEquals(expectedZ, tuple.getZ(), getEpsilon());
      }
   }

   @Test
   public void testSetOther()
   {
      Random random = new Random(621541L);
      for (int i = 0; i < 1000; ++i)
      {
         double alpha = random.nextDouble(0.0, 1.0);
         AlphaFilteredTuple3D tuple = createRandomTuple(random);
         tuple.setAlpha(alpha);

         double originalX = tuple.getX();
         double originalY = tuple.getY();
         double originalZ = tuple.getZ();

         Point3D point = new Point3D(createRandomTuple(random));
         tuple.set(point);

         double expectedX = (1.0 - alpha) * point.getX() + alpha * originalX;
         double expectedY = (1.0 - alpha) * point.getY() + alpha * originalY;
         double expectedZ = (1.0 - alpha) * point.getZ() + alpha * originalZ;

         assertEquals(expectedX, tuple.getX(), getEpsilon());
         assertEquals(expectedY, tuple.getY(), getEpsilon());
         assertEquals(expectedZ, tuple.getZ(), getEpsilon());
      }
   }

   @Override
   public AlphaFilteredTuple3D createEmptyTuple()
   {
      return new AlphaFilteredTuple3D(0.0);
   }

   @Override
   public AlphaFilteredTuple3D createTuple(double v, double v1, double v2)
   {
      return new AlphaFilteredTuple3D(v, v1, v2, 0.0);
   }

   @Override
   public AlphaFilteredTuple3D createRandomTuple(Random random)
   {
      return new AlphaFilteredTuple3D(random.nextDouble(), random.nextDouble(), random.nextDouble(), 0.0);
   }

   @Override
   public double getEpsilon()
   {
      return Epsilons.ONE_TEN_MILLIONTH;
   }
}
