package us.ihmc.stateEstimation.invariant_estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.Random;

/**
 * Tests for {@link InvariantState}: the identity constructor and sizes, set/get round-trips for
 * every named component, independence of the named columns, contact-index bounds, the tangent
 * index layout, and setToIdentity.
 */
public class InvariantStateTest
{
   private static final double EPSILON = 1.0e-12;
   private static final int ITERATIONS = 500;

   /**
    * Fresh state: X = identity, P = 0, and sizes n = 5 + N, m = 9 + 3N, for several N.
    */
   @Test
   public void testConstructorIdentityAndSizes()
   {
      int[] contactCounts = {0, 1, 2, 4};

      for (int numberOfContacts : contactCounts)
      {
         InvariantState state = new InvariantState(numberOfContacts);

         assertEquals(numberOfContacts, state.getNumberOfContacts());
         assertEquals(5 + numberOfContacts, state.getGroupSize());
         assertEquals(9 + 3 * numberOfContacts, state.getTangentSize());

         //check that state is identity group element and covariance is not-initialized yet
         assertTrue(MatrixFeatures_DDRM.isIdentity(state.getGroupElement(), EPSILON));
         assertTrue(MatrixFeatures_DDRM.isZeros(state.getCovariance(), EPSILON));
      }
   }

   /** Rotation set/get round-trip. */
   @Test
   public void testRotationRoundTrip()
   {
      Random random = new Random(1234L);
      InvariantState state = new InvariantState(2);
      RotationMatrix actual = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix expected = EuclidCoreRandomTools.nextRotationMatrix(random);
         state.setRotation(expected);
         state.getRotation(actual);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

   }

   /** Base velocity and base position set/get round-trips. */
   @Test
   public void testBaseVelocityAndPositionRoundTrip()
   {
      Random random = new Random(2345L);
      InvariantState state = new InvariantState(2);
      Vector3D velocityOut = new Vector3D();
      Vector3D positionOut = new Vector3D();

     for (int i = 0; i < ITERATIONS; i++)
     {
        Vector3D velocity = EuclidCoreRandomTools.nextVector3D(random);
        Vector3D position = EuclidCoreRandomTools.nextVector3D(random);

        state.setBaseVelocity(velocity);
        state.setBasePosition(position);

        state.getBaseVelocity(velocityOut);
        state.getBasePosition(positionOut);

        EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, velocityOut, EPSILON);
        EuclidCoreTestTools.assertVector3DGeometricallyEquals(position, positionOut, EPSILON);
     }
   }

   /**
    * Setting rotation, base velocity, base position, and every contact to distinct values,
    * then reading them all back, must return each unchanged (no column aliasing).
    */
   @Test
   public void testNamedComponentsAreIndependent()
   {
      Random random = new Random(3456L);
      int numberOfContacts = 3;
      InvariantState state = new InvariantState(numberOfContacts);

      RotationMatrix rotation = EuclidCoreRandomTools.nextRotationMatrix(random);
      Vector3D velocity = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D position = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D[] contacts = new Vector3D[numberOfContacts];
      for (int i = 0; i < numberOfContacts; i++)
         contacts[i] = EuclidCoreRandomTools.nextVector3D(random);

      state.setRotation(rotation);
      state.setBaseVelocity(velocity);
      state.setBasePosition(position);
      for (int i = 0; i < numberOfContacts; i++)
         state.setContactPosition(i, contacts[i]);

      RotationMatrix rotationOut = new RotationMatrix();
      Vector3D velocityOut = new Vector3D();
      Vector3D positionOut = new Vector3D();
      state.getRotation(rotationOut);
      state.getBaseVelocity(velocityOut);
      state.getBasePosition(positionOut);

      EuclidCoreTestTools.assertMatrix3DEquals(rotation, rotationOut, EPSILON);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, velocityOut, EPSILON);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(position, positionOut, EPSILON);

      Vector3D contactOut = new Vector3D();
      for (int i = 0; i < numberOfContacts; i++)
      {
         state.getContactPosition(i, contactOut);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(contacts[i], contactOut, EPSILON);
      }

   }

   /** Out-of-range contact indices throw IndexOutOfBoundsException on get/set/tangentIndex. */
   @Test
   public void testContactIndexOutOfBounds()
   {
      InvariantState state = new InvariantState(2);
      Vector3D out = new Vector3D();
      Vector3D in = new Vector3D();

      assertThrows(IndexOutOfBoundsException.class, () -> state.getContactPosition(-1, out));
      assertThrows(IndexOutOfBoundsException.class, () -> state.getContactPosition(2, out));
      assertThrows(IndexOutOfBoundsException.class, () -> state.setContactPosition(2, in));
      assertThrows(IndexOutOfBoundsException.class, () -> state.contactTangentIndex(2));
   }

   /** Tangent block offsets: rotation 0, velocity 3, position 6, contact i at 9 + 3i. */
   @Test
   public void testTangentIndices()
   {
      InvariantState state = new InvariantState(3);

      assertEquals(0, state.rotationTangentIndex());
      assertEquals(3, state.baseVelocityTangentIndex());
      assertEquals(6, state.basePositionTangentIndex());
      assertEquals(9, state.contactTangentIndex(0));
      assertEquals(12, state.contactTangentIndex(1));
      assertEquals(15, state.contactTangentIndex(2));
   }

   /** After mutating the state, setToIdentity restores X to the identity. */
   @Test
   public void testSetToIdentity()
   {
      Random random = new Random(4567L);
      InvariantState state = new InvariantState(2);

      state.setRotation(EuclidCoreRandomTools.nextRotationMatrix(random));
      state.setBaseVelocity(EuclidCoreRandomTools.nextVector3D(random));
      state.setBasePosition(EuclidCoreRandomTools.nextVector3D(random));
      state.setContactPosition(0,EuclidCoreRandomTools.nextVector3D(random));
      state.setContactPosition(1,EuclidCoreRandomTools.nextVector3D(random));

      state.setToIdentity();

      assertTrue(MatrixFeatures_DDRM.isIdentity(state.getGroupElement(), EPSILON));
   }
}
