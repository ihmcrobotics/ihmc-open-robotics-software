package us.ihmc.parameterEstimation.inertial;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.tools.MecanoRandomTools;

import java.util.Arrays;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;

public class RigidBodyInertialParametersTest
{
   private static final double EPSILON = 1.0e-6;

   private static final int ITERATIONS = 1000;

   ReferenceFrame bodyFrame = ReferenceFrame.getWorldFrame();
   ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();

   double mass = 1.0;
   Vector3D centerOfMassOffset = new Vector3D(0.0, 0.0, 0.0);
   Matrix3D momentOfInertia = new Matrix3D(new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
   SpatialInertia spatialInertia = new SpatialInertia(bodyFrame, expressedInFrame, momentOfInertia, mass, centerOfMassOffset);

   @Test
   public void testConstructorFromSpatialInertia()
   {
      // When the RigidBodyInertialParameters object is constructed with a SpatialInertia, it automatically performs a conversion from pi basis. Therefore,
      // we'll first perform a check that the parameter vector is constructed correctly in the default pi basis, then we'll check that the automatic conversion
      // to theta basis is what we expect
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);

      DMatrixRMaj expectedPiBasis = new DMatrixRMaj(new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0});
      DMatrixRMaj actualPiBasis = new DMatrixRMaj(expectedPiBasis.numRows, expectedPiBasis.numCols);
      assertTrue(parameterVector.isPiBasisUpToDate());
      parameterVector.getParameterVectorPiBasis(actualPiBasis);
      assertArrayEquals(expectedPiBasis.getData(), actualPiBasis.getData(), EPSILON);

      DMatrixRMaj expectedThetaBasis = new DMatrixRMaj(new double[] {0.0, -0.3465735902799726, -0.34657359027997275, -0.34657359027997275, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0});
      DMatrixRMaj actualThetaBasis = new DMatrixRMaj(expectedPiBasis.numRows, expectedPiBasis.numCols);
      parameterVector.getParameterVectorThetaBasis(actualThetaBasis);
      assertTrue(parameterVector.isThetaBasisUpToDate());
      assertArrayEquals(expectedThetaBasis.getData(), actualThetaBasis.getData(), EPSILON);
   }

   @Test
   public void testNoNaNs()
   {
      Random random = new Random(23);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         RigidBodyInertialParameters parameters = new RigidBodyInertialParameters(spatialInertia);

         DMatrixRMaj piBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
         DMatrixRMaj thetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);

         // Artificially set the theta basis as stale, update, and check for NaNs
         parameters.isThetaBasisUpToDate = false;
         parameters.update();
         parameters.getParameterVectorThetaBasis(thetaBasis);
         assertFalse(MatrixFeatures_DDRM.hasNaN(thetaBasis));

         // Artificially set the pi basis as stale, update, and check for NaNs
         parameters.isPiBasisUpToDate = false;
         parameters.update();
         parameters.getParameterVectorPiBasis(piBasis);
         assertFalse(MatrixFeatures_DDRM.hasNaN(piBasis));
      }
   }

   @Test
   public void testUpdatePiToTheta()
   {
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);

      // When the RigidBodyInertialParameters object is constructed with a SpatialInertia, it automatically performs a conversion from pi basis. Therefore, calling
      // update() should have no further effect as we haven't yet changed the parameter vector at all
      parameterVector.update();
      DMatrixRMaj expected = new DMatrixRMaj(new double[] {0.0, -0.3465735902799726, -0.34657359027997275, -0.34657359027997275, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0});
      DMatrixRMaj actual = new DMatrixRMaj(expected.numRows, expected.numCols);
      parameterVector.getParameterVectorThetaBasis(actual);
      assertTrue(parameterVector.isThetaBasisUpToDate());
      assertArrayEquals(expected.getData(), actual.getData(), EPSILON);

      // Now we set a new value for the parameter vector in pi basis -- but if we don't call update(), we should get the same result as expected for the
      // original parameter setting
      parameterVector.setParameterVectorPiBasis(new DMatrixRMaj(new double[] {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 2.0, 0.0, 2.0}));
      expected.set(new DMatrixRMaj(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      parameterVector.getParameterVectorThetaBasis(actual);
      assertFalse(parameterVector.isThetaBasisUpToDate());
      assertFalse(Arrays.equals(expected.getData(), actual.getData()));

      // Now let's update the theta basis with the new parameter vector setting
      parameterVector.update();
      parameterVector.getParameterVectorThetaBasis(actual);
      assertTrue(parameterVector.isThetaBasisUpToDate());
      assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
   }

   @Test
   public void testUpdateThetaToPi()
   {
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);

      // Set a new value in theta basis, but do not call update() -- we should get a different answer than what we expect in the pi basis
      parameterVector.setParameterVectorThetaBasis(new DMatrixRMaj(new double[] {0.5493061443340548, -0.8958797346140274, -0.8958797346140275, -0.8958797346140275, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0}));
      DMatrixRMaj expected = new DMatrixRMaj(new double[] {3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0});
      DMatrixRMaj actual = new DMatrixRMaj(expected.numRows, expected.numCols);
      parameterVector.getParameterVectorPiBasis(actual);
      assertFalse(parameterVector.isPiBasisUpToDate());
      assertFalse(Arrays.equals(expected.getData(), actual.getData()));

      // Now let's update the pi basis with the new parameter vector setting
      parameterVector.update();
      parameterVector.getParameterVectorPiBasis(actual);
      assertTrue(parameterVector.isPiBasisUpToDate());
      assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
   }

   @Test
   public void testStaticConversionKnownPiToKnownTheta()
   {
      DMatrixRMaj piBasis = new DMatrixRMaj(new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0});
      DMatrixRMaj thetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY);
      RigidBodyInertialParameters.fromPiBasisToThetaBasis(piBasis, thetaBasis);

      DMatrixRMaj expectedThetaBasis = new DMatrixRMaj(new double[] {0.0, -0.3465735902799726, -0.34657359027997275, -0.34657359027997275, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0});
      assertArrayEquals(expectedThetaBasis.getData(), thetaBasis.getData(), EPSILON);
   }

   @Test
   public void testStaticConversionKnownThetaToKnownPi()
   {
      DMatrixRMaj thetaBasis = new DMatrixRMaj(new double[] {0.0, -0.3465735902799726, -0.34657359027997275, -0.34657359027997275, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0});
      DMatrixRMaj piBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY);
      RigidBodyInertialParameters.fromThetaBasisToPiBasis(thetaBasis, piBasis);

      DMatrixRMaj piBasisExpected = new DMatrixRMaj(new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0});
      assertArrayEquals(piBasisExpected.getData(), piBasis.getData(), EPSILON);
   }

   @Test
   public void testStaticConversionPreservesParameters()
   {
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);

      // Check that converting from pi to theta and back to pi gets the same results
      DMatrixRMaj parameterVectorPiBasisBefore = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      parameterVector.getParameterVectorPiBasis(parameterVectorPiBasisBefore);
      DMatrixRMaj parameterVectorThetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      RigidBodyInertialParameters.fromPiBasisToThetaBasis(parameterVectorPiBasisBefore, parameterVectorThetaBasis);
      DMatrixRMaj parameterVectorPiBasisAfter = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      RigidBodyInertialParameters.fromThetaBasisToPiBasis(parameterVectorThetaBasis, parameterVectorPiBasisAfter);

      assertArrayEquals(parameterVectorPiBasisBefore.getData(), parameterVectorPiBasisAfter.getData(), EPSILON);
   }

   @Test
   public void testUpdateWithNoTrusted()
   {
      // This shouldn't occur in normal operation because the API of RigidBodyInertialParameters is purposefully crafted such that it is impossible for both
      // the pi basis and theta basis to be out of date, but here we test that nothing is updated if both of the bases are out of date, as neither can be trusted
      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);

      // Update parameter vector in pi basis (which should set isThetaBasisUpToDate to false), but then also set isPiBasisUpToDate to false as well
      parameterVector.setParameterVectorPiBasis(new DMatrixRMaj(new double[] {3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0}));
      parameterVector.update();

      DMatrixRMaj piBasisBefore = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      DMatrixRMaj thetaBasisBefore = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      parameterVector.getParameterVectorPiBasis(piBasisBefore);
      parameterVector.getParameterVectorThetaBasis(thetaBasisBefore);

      parameterVector.isPiBasisUpToDate = false;
      parameterVector.update();

      DMatrixRMaj piBasisAfter = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      DMatrixRMaj thetaBasisAfter = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      parameterVector.getParameterVectorPiBasis(piBasisAfter);
      parameterVector.getParameterVectorThetaBasis(thetaBasisAfter);

      assertArrayEquals(piBasisBefore.getData(), piBasisAfter.getData(), EPSILON);
      assertArrayEquals(thetaBasisBefore.getData(), thetaBasisAfter.getData(), EPSILON);
   }

   @Test
   public void testUpdatePiBasisRepetitive()
   {
      // Testing conversion from pi basis to theta basis is difficult to do programmatically without relying on another reference implementation of the
      // conversion logic. Here, we keep the mass fixed to one, the center of mass offset fixed to zero, and randomly generate positive definite 3D matrices
      // for the moment of inertia. With this configuration of the pi basis, only the d1, d2, d3, s12, s13, s23 components in the theta basis vary, with the rest being zero.
      Random random = new Random(23);

      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);

      DMatrixRMaj thetaBasisPrevious = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      DMatrixRMaj thetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      for (int i = 0; i < ITERATIONS; ++i)
      {
         // We use a small window to ensure that the full physical consistency condition of the triangle inequality being satisfied on the principal moments of
         // inertia is true
         double minDiagonal = 0.1;
         double maxDiagonal = 0.2;
         spatialInertia.getMomentOfInertia().set(EuclidCoreRandomTools.nextDiagonalMatrix3D(random, minDiagonal, maxDiagonal));
         parameterVector.setParameterVectorPiBasis(spatialInertia);

         parameterVector.getParameterVectorThetaBasis(thetaBasisPrevious);
         parameterVector.update();
         parameterVector.getParameterVectorThetaBasis(thetaBasis);

         // Is everything up-to-date?
         assertTrue(parameterVector.isPiBasisUpToDate());
         assertTrue(parameterVector.isThetaBasisUpToDate());
         // Are all components apart from d1, d2, d3, s12, s13, s23 of theta basis zero?
         assertEquals(0.0, thetaBasis.get(0));
         assertEquals(0.0, thetaBasis.get(7));
         assertEquals(0.0, thetaBasis.get(8));
         assertEquals(0.0, thetaBasis.get(9));
         // Is the theta basis from the previous random draw different from this draw? (should be probability approaching zero of them being the same
         assertFalse(Arrays.equals(thetaBasisPrevious.getData(), thetaBasis.getData()));
      }
   }

   @Test
   public void testUpdateThetaBasisRepetitive()
   {
      // Testing conversion from theta basis to pi basis is difficult to do programmatically without relying on another reference implementation of the
      // conversion logic. Here, we keep all theta basis parameters as zero apart from alpha, for which we randomly generate positive doubles. With this
      // configuration of the theta basis, only the mass and the diagonal components of the moment of inertia in the pi basis vary, with the rest being zero.
      Random random = new Random(23);

      RigidBodyInertialParameters parameterVector = new RigidBodyInertialParameters(spatialInertia);
      DMatrixRMaj thetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);

      DMatrixRMaj piBasisPrevious = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      DMatrixRMaj piBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      for (int i = 0; i < ITERATIONS; ++i)
      {
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.1, 1.0);
         thetaBasis.set(0, alpha);
         parameterVector.setParameterVectorThetaBasis(thetaBasis);

         parameterVector.getParameterVectorPiBasis(piBasisPrevious);
         parameterVector.update();
         parameterVector.getParameterVectorPiBasis(piBasis);

         // Is everything up-to-date?
         assertTrue(parameterVector.isThetaBasisUpToDate());
         assertTrue(parameterVector.isPiBasisUpToDate());
         // Are all components apart from m, Ixx, Iyy, Izz of pi basis zero?
         assertEquals(0.0, thetaBasis.get(1));
         assertEquals(0.0, thetaBasis.get(2));
         assertEquals(0.0, thetaBasis.get(3));
         assertEquals(0.0, thetaBasis.get(5));
         assertEquals(0.0, thetaBasis.get(7));
         assertEquals(0.0, thetaBasis.get(8));
         // Is the theta basis from the previous random draw different from this draw? (should be probability approaching zero of them being the same
         assertFalse(Arrays.equals(piBasisPrevious.getData(), piBasis.getData()));
      }
   }

   @Test
   public void testFromThetaBasisToPiBasisJacobianOnKnownExample()
   {
      // This setting for theta should always result in a pi of [1, 0, 0, 0, 1, 0, 1, 0, 0, 1] (unit mass, identity inertia, zero center of mass offset)
      DMatrixRMaj theta = new DMatrixRMaj(new double[] {0.0, -0.3465735902799726, -0.34657359027997275, -0.34657359027997275, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0});

      DMatrixRMaj pi = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      RigidBodyInertialParameters.fromThetaBasisToPiBasis(theta, pi);

      DMatrixRMaj expectedPi = new DMatrixRMaj(new double[] {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0});
      assertArrayEquals(expectedPi.getData(), pi.getData(), EPSILON);

      DMatrixRMaj jacobian = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, PARAMETERS_PER_RIGID_BODY);
      RigidBodyInertialParameters.fromThetaBasisToPiBasisJacobian(theta, jacobian);

      // Furthermore, it should always result in the following Jacobian
      DMatrixRMaj expectedJacobian = new DMatrixRMaj(new double[][] {
            {2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.70710677, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.70710677, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.70710677},
            {2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, -0.70710677, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, -0.70710677, 0.0, 0.0, 0.0, 0.0},
            {2.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.70710677, 0.0, 0.0, 0.0},
            {2.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
      });
      assertArrayEquals(expectedJacobian.getData(), jacobian.getData(), EPSILON);
   }

   @Test
   public void testFromThetaBasisToPiBasisJacobianIsAlwaysFullRank()
   {
      Random random = new Random(23);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);

         RigidBodyInertialParameters parameters = new RigidBodyInertialParameters(spatialInertia);

         DMatrixRMaj parameterVectorThetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
         parameters.getParameterVectorThetaBasis(parameterVectorThetaBasis);

         DMatrixRMaj jacobian = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, PARAMETERS_PER_RIGID_BODY);
         RigidBodyInertialParameters.fromThetaBasisToPiBasisJacobian(parameterVectorThetaBasis, jacobian);

         assertEquals(MatrixFeatures_DDRM.rank(jacobian), PARAMETERS_PER_RIGID_BODY);
      }
   }
}
