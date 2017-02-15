package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class SpatialMotionVectorTest
{
   protected Random random = new Random(100L);

   protected ReferenceFrame frameA;
   protected ReferenceFrame frameB;
   protected ReferenceFrame frameC;
   protected ReferenceFrame frameD;


   @Before
   public void setUp() throws Exception
   {
      frameA = ReferenceFrame.constructAWorldFrame("A");
      frameB = new ReferenceFrame("B", frameA)
      {
         private static final long serialVersionUID = 1L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
            RigidBodyTransform translation = new RigidBodyTransform();
            translation.setTranslation(new Vector3d(3.0, 4.0, 5.0));
            transformToParent.multiply(translation);
         }
      };

      frameC = new ReferenceFrame("C", frameB)
      {
         private static final long serialVersionUID = 1L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
            RigidBodyTransform translation = new RigidBodyTransform();
            translation.setTranslation(new Vector3d(3.0, 4.0, 5.0));
            transformToParent.multiply(translation);
         }
      };

      frameD = ReferenceFrame.constructAWorldFrame("D");

      frameB.update();
      frameC.update();
   }

   /**
    * Test inverting a twist
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInvert()
   {
      Vector3d linearPart = RandomTools.generateRandomVector(random);
      Vector3d angularPart = RandomTools.generateRandomVector(random);

      Vector3d linearPartInverse = new Vector3d(linearPart);
      linearPartInverse.scale(-1.0);

      Vector3d angularPartInverse = new Vector3d(angularPart);
      angularPartInverse.scale(-1.0);

      SpatialMotionVector twist1 = createSpatialMotionVector(frameB, frameA, frameA, linearPart, angularPart);
      twist1.invert();

      double epsilon = 1e-10;
      JUnitTools.assertTuple3dEquals(angularPartInverse, twist1.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(linearPartInverse, twist1.getLinearPartCopy(), epsilon);
      assertEquals(frameA, twist1.getExpressedInFrame());
      assertEquals(frameB, twist1.getBaseFrame());
      assertEquals(frameA, twist1.getBodyFrame());

      SpatialMotionVector twist2 = createSpatialMotionVector(frameB, frameA, frameB, linearPart, angularPart);
      twist2.invert();
      JUnitTools.assertTuple3dEquals(angularPartInverse, twist2.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(linearPartInverse, twist2.getLinearPartCopy(), epsilon);
      assertEquals(frameB, twist2.getExpressedInFrame());
      assertEquals(frameB, twist2.getBaseFrame());
      assertEquals(frameA, twist2.getBodyFrame());
   }

   /**
    * Constructing using a matrix
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructUsingMatrix()
   {
      DenseMatrix64F matrix = RandomMatrices.createRandom(SpatialMotionVector.SIZE, 1, random);
      SpatialMotionVector spatialMotionVector = createSpatialMotionVector(frameC, frameD, frameA, matrix);
      DenseMatrix64F matrixBack = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
      spatialMotionVector.getMatrix(matrixBack, 0);
      JUnitTools.assertMatrixEquals(matrix, matrixBack, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testConstructUsingMatrixTooSmall()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(SpatialMotionVector.SIZE - 1, 1);
      createSpatialMotionVector(frameC, frameD, frameA, matrix);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testConstructUsingMatrixTooBig()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(SpatialMotionVector.SIZE + 1, 1);
      createSpatialMotionVector(frameC, frameD, frameA, matrix);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testConstructUsingMatrixTooBig2()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 2);
      createSpatialMotionVector(frameC, frameD, frameA, matrix);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLimitLinearAndAngularParts()
   {
      double linearLength = 10.0;
      double angularLength = 6.3;

      Vector3d linearPart = RandomTools.generateRandomVector(random, linearLength);
      Vector3d angularPart = RandomTools.generateRandomVector(random, angularLength);
      
      SpatialMotionVector vector = createSpatialMotionVector(frameB, frameA, frameA, linearPart, angularPart);
            
      double maximumLinearMagnitude = 100.0;
      double maximumAngularMagnitude = 72.0;

      vector.limitLinearPartMagnitude(maximumLinearMagnitude);
      vector.limitAngularPartMagnitude(maximumAngularMagnitude);
            
      assertEquals(linearLength, vector.getLinearPartCopy().length(), 1e-7);
      assertEquals(angularLength, vector.getAngularPartCopy().length(), 1e-7);
      
      maximumLinearMagnitude = 1.9;
      maximumAngularMagnitude = 0.4;
      
      vector.limitLinearPartMagnitude(maximumLinearMagnitude);
      vector.limitAngularPartMagnitude(maximumAngularMagnitude);
            
      assertEquals(maximumLinearMagnitude, vector.getLinearPartCopy().length(), 1e-7);
      assertEquals(maximumAngularMagnitude, vector.getAngularPartCopy().length(), 1e-7);
            
   }

   protected SpatialMotionVector createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
           Vector3d linearPart, Vector3d angularPart)
   {
      return new GenericSpatialMotionVector(bodyFrame, baseFrame, expressedInFrame, linearPart, angularPart);
   }

   protected SpatialMotionVector createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
           DenseMatrix64F matrix)
   {
      return new GenericSpatialMotionVector(bodyFrame, baseFrame, expressedInFrame, matrix);
   }

   public class GenericSpatialMotionVector extends SpatialMotionVector
   {
      public GenericSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3d linearPart,
                                        Vector3d angularPart)
      {
         this.angularPart = new Vector3d();
         this.linearPart = new Vector3d();
         set(bodyFrame, baseFrame, expressedInFrame, linearPart, angularPart);
      }

      /**
       * Construct using a Matrix ([omega; v])
       */
      public GenericSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F twistMatrix)
      {
         MatrixTools.checkMatrixDimensions(twistMatrix, SIZE, 1);
         this.angularPart = new Vector3d();
         this.linearPart = new Vector3d();
         set(bodyFrame, baseFrame, expressedInFrame, twistMatrix, 0);
      }
   }

   public static void assertSpatialMotionVectorEquals(SpatialMotionVector vector1, SpatialMotionVector vector2, double epsilon)
   {
      assertSpatialMotionVectorEquals(vector1, vector2, null, epsilon);
   }

   public static void assertSpatialMotionVectorEquals(SpatialMotionVector vector1, SpatialMotionVector vector2, DenseMatrix64F selectionMatrix, double epsilon)
   {
      if (selectionMatrix != null)
      {
         DenseMatrix64F motionVectorOne = new DenseMatrix64F(6, 1);
         vector1.getMatrix(motionVectorOne, 0);
         DenseMatrix64F selectedVectorOne = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);

         DenseMatrix64F motionVectorTwo = new DenseMatrix64F(6, 1);
         vector2.getMatrix(motionVectorTwo, 0);
         DenseMatrix64F selectedVectorTwo = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);

         CommonOps.mult(selectionMatrix, motionVectorOne, selectedVectorOne);
         CommonOps.mult(selectionMatrix, motionVectorTwo, selectedVectorTwo);

         JUnitTools.assertMatrixEquals(selectedVectorOne, selectedVectorTwo, epsilon);
      }

      else
      {
         assertEquals(vector1.getBodyFrame(), vector2.getBodyFrame());
         assertEquals(vector1.getBaseFrame(), vector2.getBaseFrame());
         assertEquals(vector1.getExpressedInFrame(), vector2.getExpressedInFrame());

         JUnitTools.assertTuple3dEquals(vector1.getAngularPartCopy(), vector2.getAngularPartCopy(), epsilon);
         JUnitTools.assertTuple3dEquals(vector1.getLinearPartCopy(), vector2.getLinearPartCopy(), epsilon);
      }
   }
}
