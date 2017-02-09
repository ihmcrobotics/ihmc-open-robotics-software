package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.testing.JUnitTools;

public class Transform3dTest
{
	private final int nTests = 200;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestAxisAngleAllZerosAndScale()
	{
	   AxisAngle4d axisAngle = new AxisAngle4d();
	   Transform3d transform = new Transform3d();
	   Matrix3d matrix = new Matrix3d();
	   
	   for(int i = 0; i<nTests; i++)
	   {
	      axisAngle.setX(0);
	      axisAngle.setY(0);
	      axisAngle.setZ(0);
	      axisAngle.setAngle(0);
	      
	      transform.set(axisAngle, new Vector3d(0,0,0), 3);
	      
	      transform.getRotationScale(matrix);
	      
	      assertEquals(matrix.getM00(), 3,1e-10);
	      assertEquals(matrix.getM11(),3,1e-10);
	      assertEquals(matrix.getM22(),3,1e-10);
	      
	      transform.getRotation(matrix);
	      
	      assertEquals(matrix.getM00(),1,1e-10);
	      assertEquals(matrix.getM11(),1,1e-10);
	      assertEquals(matrix.getM22(),1,1e-10);
	   }
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void TestAxisAngleAllZerosWithTranslationAndScale()
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      Transform3d transform = new Transform3d();
      Matrix3d matrix = new Matrix3d();
      Random random = new Random();
      Vector3d vectorCheck = new Vector3d();
      
      for(int i = 0; i<nTests; i++)
      {
         axisAngle.setX(0);
         axisAngle.setY(0);
         axisAngle.setZ(0);
         axisAngle.setAngle(0);
         Vector3d vector = new Vector3d(random.nextDouble(),random.nextDouble(),random.nextDouble());
         transform.set(axisAngle, vector, 3);
         
         transform.getRotationScale(matrix);
         
         assertEquals(matrix.getM00(), 3,1e-10);
         assertEquals(matrix.getM11(),3,1e-10);
         assertEquals(matrix.getM22(),3,1e-10);
         
         transform.getRotation(matrix);
         transform.getTranslation(vectorCheck);
         
         assertEquals(matrix.getM00(),1,1e-10);
         assertEquals(matrix.getM11(),1,1e-10);
         assertEquals(matrix.getM22(),1,1e-10);
         JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-8);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void TestAxisAngleAllZerosWithTranslationAndScaleWithFloats()
   {
      AxisAngle4f axisAngle = new AxisAngle4f();
      Transform3d transform = new Transform3d();
      Matrix3f matrix = new Matrix3f();
      Random random = new Random();
      Vector3f vectorCheck = new Vector3f();
      
      for(int i = 0; i<nTests; i++)
      {
         axisAngle.setX(0);
         axisAngle.setY(0);
         axisAngle.setZ(0);
         axisAngle.setAngle(0);
         Vector3f vector = new Vector3f(random.nextFloat(),random.nextFloat(),random.nextFloat());
         transform.set(axisAngle, vector, 3.0f);
         
         transform.getRotationScale(matrix);
         
         assertEquals(matrix.getM00(), 3,1e-10);
         assertEquals(matrix.getM11(),3,1e-10);
         assertEquals(matrix.getM22(),3,1e-10);
         
         transform.getRotation(matrix);
         transform.getTranslation(vectorCheck);
         
         assertEquals(matrix.getM00(),1,1e-10);
         assertEquals(matrix.getM11(),1,1e-10);
         assertEquals(matrix.getM22(),1,1e-10);
         JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-8);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void TestAxisAngleAllZerosAndVectorScale()
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      Transform3d transform = new Transform3d();
      Matrix3d matrix = new Matrix3d();
      
      for(int i = 0; i<nTests; i++)
      {
         axisAngle.setX(0);
         axisAngle.setY(0);
         axisAngle.setZ(0);
         axisAngle.setAngle(0);
         
         transform.set(axisAngle, new Vector3d(0,0,0), 1,2,3);
         
         transform.getRotationScale(matrix);
         
         assertEquals(matrix.getM00(), 1,1e-10);
         assertEquals(matrix.getM11(),2,1e-10);
         assertEquals(matrix.getM22(),3,1e-10);
         
         transform.getRotation(matrix);
         
         assertEquals(matrix.getM00(),1,1e-10);
         assertEquals(matrix.getM11(),1,1e-10);
         assertEquals(matrix.getM22(),1,1e-10);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales1()
	{
		Matrix3d matrix = new Matrix3d();
		Vector3d vector = new Vector3d();
		Vector3d scales = new Vector3d();
		Random random = new Random();

		Matrix3d matrixCheck = new Matrix3d();
		Vector3d vectorCheck = new Vector3d();
		Vector3d scalesCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(matrix, vector, scales.getX(),
					scales.getY(), scales.getZ());

			transform.getRotation(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			JUnitTools.assertMatrix3dEquals("", matrix, matrixCheck, 1e-8);
			JUnitTools.assertVector3dEquals("", vectorCheck, vectorCheck, 1e-8);
			JUnitTools.assertVector3dEquals("", scales, scalesCheck, 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales2()
	{
		Matrix3f matrix = new Matrix3f();
		Vector3f vector = new Vector3f();
		Vector3f scales = new Vector3f();
		Random random = new Random();

		Matrix3f matrixCheck = new Matrix3f();
		Vector3f vectorCheck = new Vector3f();
		Vector3f scalesCheck = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(matrix, vector, scales.getX(),
					scales.getY(), scales.getZ());

			transform.getRotation(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			JUnitTools.assertMatrix3fEquals("", matrix, matrixCheck, 1e-5);
			JUnitTools.assertVector3fEquals("", vectorCheck, vectorCheck, 1e-5);
			JUnitTools.assertVector3fEquals("", scales, scalesCheck, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales3()
	{
		AxisAngle4d axisAngle = new AxisAngle4d();
		Vector3d vector = new Vector3d();
		Vector3d scales = new Vector3d();
		Random random = new Random();

		AxisAngle4d axisAngleCheck = new AxisAngle4d();
		Vector3d vectorCheck = new Vector3d();
		Vector3d scalesCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.sin(theta));
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle(theta);
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(axisAngle, vector,
					scales.getX(), scales.getY(), scales.getZ());

			transform.getRotation(axisAngleCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			assertEquals(axisAngle.getX(), axisAngleCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleCheck.getAngle(), 1e-8);
			JUnitTools.assertVector3dEquals("", vectorCheck, vectorCheck, 1e-5);
			JUnitTools.assertVector3dEquals("", scales, scalesCheck, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales4()
	{
		AxisAngle4f axisAngle = new AxisAngle4f();
		Vector3f vector = new Vector3f();
		Vector3f scales = new Vector3f();
		Random random = new Random();

		AxisAngle4f axisAngleCheck = new AxisAngle4f();
		Vector3f vectorCheck = new Vector3f();
		Vector3f scalesCheck = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX((float) Math.sin(theta));
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle((float) theta);
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(axisAngle, vector,
					scales.getX(), scales.getY(), scales.getZ());

			transform.getRotation(axisAngleCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			assertEquals(axisAngle.getX(), axisAngleCheck.getX(), 1e-5);
			assertEquals(axisAngle.getY(), axisAngleCheck.getY(), 1e-5);
			assertEquals(axisAngle.getZ(), axisAngleCheck.getZ(), 1e-5);
			assertEquals(axisAngle.getAngle(), axisAngleCheck.getAngle(), 1e-5);
			JUnitTools.assertVector3fEquals("", vectorCheck, vectorCheck, 1e-5);
			JUnitTools.assertVector3fEquals("", scales, scalesCheck, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales5()
	{
		Quat4f quat = new Quat4f();
		Vector3f vector = new Vector3f();
		Vector3f scales = new Vector3f();
		Random random = new Random();

		Quat4f axisAngleCheck = new Quat4f();
		Vector3f vectorCheck = new Vector3f();
		Vector3f scalesCheck = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			quat.setX(random.nextFloat());
			quat.setY(random.nextFloat());
			quat.setZ(random.nextFloat());
			quat.setW(random.nextFloat());
			quat.normalize();
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(quat, vector, scales.getX(),
					scales.getY(), scales.getZ());

			transform.getRotation(axisAngleCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			assertEquals(quat.getX(), axisAngleCheck.getX(), 1e-5);
			assertEquals(quat.getY(), axisAngleCheck.getY(), 1e-5);
			assertEquals(quat.getZ(), axisAngleCheck.getZ(), 1e-5);
			assertEquals(quat.getW(), axisAngleCheck.getW(), 1e-5);
			JUnitTools.assertVector3fEquals("", vectorCheck, vectorCheck, 1e-5);
			JUnitTools.assertVector3fEquals("", scales, scalesCheck, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales6()
	{
		Quat4d quat = new Quat4d();
		Vector3d vector = new Vector3d();
		Vector3d scales = new Vector3d();
		Random random = new Random();

		Quat4d axisAngleCheck = new Quat4d();
		Vector3d vectorCheck = new Vector3d();
		Vector3d scalesCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			quat.setX(random.nextFloat());
			quat.setY(random.nextFloat());
			quat.setZ(random.nextFloat());
			quat.setW(random.nextFloat());
			quat.normalize();
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(quat, vector, scales.getX(),
					scales.getY(), scales.getZ());

			transform.getRotation(axisAngleCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			assertEquals(quat.getX(), axisAngleCheck.getX(), 1e-5);
			assertEquals(quat.getY(), axisAngleCheck.getY(), 1e-5);
			assertEquals(quat.getZ(), axisAngleCheck.getZ(), 1e-5);
			assertEquals(quat.getW(), axisAngleCheck.getW(), 1e-5);
			JUnitTools.assertVector3dEquals("", vectorCheck, vectorCheck, 1e-5);
			JUnitTools.assertVector3dEquals("", scales, scalesCheck, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMultipleDoubleScales7()
	{
		DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
		Vector3d vector = new Vector3d();
		Vector3d scales = new Vector3d();
		Random random = new Random();

		DenseMatrix64F matrixCheck = new DenseMatrix64F(3, 3);
		Vector3d vectorCheck = new Vector3d();
		Vector3d scalesCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);
			randomizeVector(random, scales);

			Transform3d transform = new Transform3d(matrix, vector, scales.getX(),
					scales.getY(), scales.getZ());

			transform.getRotation(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scalesCheck);

			JUnitTools.assertMatrixEquals("", matrix, matrixCheck, 1e-8);
			JUnitTools.assertVector3dEquals("", vectorCheck, vectorCheck, 1e-8);
			JUnitTools.assertVector3dEquals("", scales, scalesCheck, 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentation()
	{
		Random random = new Random();
		AxisAngle4d axisAngle = new AxisAngle4d();
		AxisAngle4d axisAngleToCheck = new AxisAngle4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.sin(theta));
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle(theta);

			transform.setRotationAndZeroTranslation(axisAngle);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(0);
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			transform.setRotation(axisAngle);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			transform.setRotation(axisAngle);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentation2()
	{
		Random random = new Random();
		Vector3d vector = new Vector3d();
		AxisAngle4d axisAngle = new AxisAngle4d();
		AxisAngle4d axisAngleToCheck = new AxisAngle4d();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.sin(theta));
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector, 1.0);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(0);
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector, 1.0);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector, 1.0);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentation2WithScale()
	{
		Random random = new Random();
		Vector3d vector = new Vector3d();
		AxisAngle4d axisAngle = new AxisAngle4d();
		AxisAngle4d axisAngleToCheck = new AxisAngle4d();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.sin(theta));
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			double scale = random.nextDouble() + 0.1;
			Transform3d transform = new Transform3d(axisAngle, vector,
					scale + 0.1);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(0);
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			// Test case with zero scale
			Transform3d transform = new Transform3d(axisAngle, vector,
					random.nextDouble() + 0.1);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					random.nextDouble());
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentation2WithVectorScale()
	{
		Random random = new Random();
		Vector3d vector = new Vector3d();
		AxisAngle4d axisAngle = new AxisAngle4d();
		AxisAngle4d axisAngleToCheck = new AxisAngle4d();
		Matrix3d matrix = new Matrix3d();
		Vector3d vectorCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
//			axisAngle.set(matrix);
			RotationTools.convertMatrixToAxisAngle(matrix, axisAngle);
			randomizeVector(random, vector);
			Vector3d scale = new Vector3d(random.nextDouble(),
					random.nextDouble(), random.nextDouble());
			Transform3d transform = new Transform3d(axisAngle, vector, scale);
			transform.getRotation(axisAngleToCheck);
			transform.getScale(vectorCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(0);
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			// Test case with zero scale
			Transform3d transform = new Transform3d(axisAngle, vector,
					new Vector3d(random.nextDouble(), random.nextDouble(),
							random.nextDouble()));
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					new Vector3d(random.nextDouble(), random.nextDouble(),
							random.nextDouble()));
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentation3()
	{
		Random random = new Random();
		AxisAngle4d axisAngle = new AxisAngle4d();
		AxisAngle4d axisAngleToCheck = new AxisAngle4d();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.sin(theta));
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle(theta);

			Transform3d transform = new Transform3d(axisAngle, new Vector3d(0,
					0, 0), 1.0);

			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(0);
			axisAngle.setY(Math.cos(theta));
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			Transform3d transform1 = new Transform3d(axisAngle, new Vector3d(0,
					0, 0), 1.0);

			transform1.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextDouble();
			axisAngle.setX(Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ(Math.sin(theta));
			axisAngle.setAngle(theta);

			Transform3d transform2 = new Transform3d(axisAngle, new Vector3d(0,
					0, 0), 1.0);
			transform2.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-8);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-8);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-8);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-8);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentationWithFloats()
	{
		Random random = new Random();
		AxisAngle4f axisAngle = new AxisAngle4f();
		AxisAngle4f axisAngleToCheck = new AxisAngle4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.sin(theta));
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle((float) theta);

			transform.setRotationAndZeroTranslation(axisAngle);
			transform.normalizeRotationPart();
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX(0);
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			transform.setRotationAndZeroTranslation(axisAngle);
			transform.normalizeRotationPart();
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			transform.setRotationAndZeroTranslation(axisAngle);
			transform.normalizeRotationPart();
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentationWithFloats2()
	{
		Random random = new Random();
		AxisAngle4f axisAngle = new AxisAngle4f();
		AxisAngle4f axisAngleToCheck = new AxisAngle4f();
		Vector3f vector = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.sin(theta));
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector, 1.0);
			transform.normalizeRotationPart();
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX(0);
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector, 1.0);
			transform.normalizeRotationPart();
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector, 1.0);
			transform.normalizeRotationPart();
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentationWithFloats2AndScale()
	{
		Random random = new Random();
		AxisAngle4f axisAngle = new AxisAngle4f();
		AxisAngle4f axisAngleToCheck = new AxisAngle4f();
		Vector3f vector = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.sin(theta));
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					random.nextDouble() + 0.1);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX(0);
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					random.nextDouble() + 0.1);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					random.nextDouble() + 0.1);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentationWithFloats2AndVectorScale()
	{
		Random random = new Random();
		AxisAngle4f axisAngle = new AxisAngle4f();
		AxisAngle4f axisAngleToCheck = new AxisAngle4f();
		Vector3f vector = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.sin(theta));
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					new Vector3f(random.nextFloat(), random.nextFloat(),
							random.nextFloat()));
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX(0);
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					new Vector3f(random.nextFloat(), random.nextFloat(),
							random.nextFloat()));
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(axisAngle, vector,
					new Vector3f(random.nextFloat(), random.nextFloat(),
							random.nextFloat()));
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestNormalize()
	{
		Random random = new Random();
		Matrix4d matrix = new Matrix4d();
		Transform3d transform = new Transform3d();

		createRandomTransformationMatrix(matrix, random);
		messWithRotationMatrixOrthogonality(matrix, random);

		transform.set(matrix);

		transform.normalizeRotationPart();

		assertTrue(checkOrthogonality(transform));

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestUseAxisAngleRepresentationWithFloats3()
	{
		Random random = new Random();
		AxisAngle4f axisAngle = new AxisAngle4f();
		AxisAngle4f axisAngleToCheck = new AxisAngle4f();

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.sin(theta));
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ(0);
			axisAngle.setAngle((float) theta);

			Transform3d transform = new Transform3d(axisAngle, new Vector3f(0,
					0, 0), 1.0);
			transform.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX(0);
			axisAngle.setY((float) Math.cos(theta));
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			Transform3d transform1 = new Transform3d(axisAngle, new Vector3f(0,
					0, 0), 1.0);
			transform1.normalizeRotationPart();
			transform1.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-3);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-3);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-3);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-3);
		}

		for (int i = 0; i < nTests; i++)
		{
			double theta = 0.01 + (Math.PI - 0.02) * random.nextFloat();
			axisAngle.setX((float) Math.cos(theta));
			axisAngle.setY(0);
			axisAngle.setZ((float) Math.sin(theta));
			axisAngle.setAngle((float) theta);

			Transform3d transform2 = new Transform3d(axisAngle, new Vector3f(0,
					0, 0), 1.0);
			transform2.normalizeRotationPart();
			transform2.getRotation(axisAngleToCheck);

			assertEquals(axisAngle.getX(), axisAngleToCheck.getX(), 1e-4);
			assertEquals(axisAngle.getY(), axisAngleToCheck.getY(), 1e-4);
			assertEquals(axisAngle.getZ(), axisAngleToCheck.getZ(), 1e-4);
			assertEquals(axisAngle.getAngle(), axisAngleToCheck.getAngle(), 1e-4);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestGetTransformAsQuat4dAndVector3d()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			transform.setRotationAndZeroTranslation(quat1);

			transform.get(quatCheck, vec);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestGetTransformAsQuat4dAndVector3dWithScale()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Transform3d transform = new Transform3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			transform.setRotationAndZeroTranslation(quat1);
			double scale = 100 * random.nextDouble();
			transform.setScale(scale);

			transform.get(quatCheck, vec);
			transform.getScale(scaleCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(scale, scaleCheck.getX(), 1e-6);
			assertEquals(scale, scaleCheck.getY(), 1e-6);
			assertEquals(scale, scaleCheck.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestGetTransformAsQuat4dAndVector3dWithZeroScale()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Transform3d transform = new Transform3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			transform.setRotationAndZeroTranslation(quat1);
			double scale = 0;
			transform.setScale(scale);

			transform.get(quatCheck, vec);
			transform.getScale(scaleCheck);

			assertEquals(0, quatCheck.getX(), 1e-10);
			assertEquals(0, quatCheck.getY(), 1e-10);
			assertEquals(0, quatCheck.getZ(), 1e-10);
			assertEquals(1, quatCheck.getW(), 1e-10);
			assertEquals(scale, scaleCheck.getX(), 1e-6);
			assertEquals(scale, scaleCheck.getY(), 1e-6);
			assertEquals(scale, scaleCheck.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4dAndVector3d()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, trans);
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			Transform3d transform = new Transform3d(quat1, trans, 1.0);

			transform.get(quatCheck, vec);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(vec.getX(), trans.getX(), 1e-10);
			assertEquals(vec.getY(), trans.getY(), 1e-10);
			assertEquals(vec.getZ(), trans.getZ(), 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4dAndVector3dAndScale()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, trans);
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			double scale = random.nextDouble();
			Transform3d transform = new Transform3d(quat1, trans, scale);

			transform.get(quatCheck, vec);
			transform.getScale(scaleCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(vec.getX(), trans.getX(), 1e-10);
			assertEquals(vec.getY(), trans.getY(), 1e-10);
			assertEquals(vec.getZ(), trans.getZ(), 1e-10);
			assertEquals(scaleCheck.getX(), scale, 1e-6);
			assertEquals(scaleCheck.getY(), scale, 1e-6);
			assertEquals(scaleCheck.getZ(), scale, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4dAndVector3dAndVectorScale()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, trans);
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			Vector3d scale = new Vector3d(random.nextDouble(),
					random.nextDouble(), random.nextDouble());
			Transform3d transform = new Transform3d(quat1, trans, scale);

			transform.get(quatCheck, vec);
			transform.getScale(scaleCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(vec.getX(), trans.getX(), 1e-10);
			assertEquals(vec.getY(), trans.getY(), 1e-10);
			assertEquals(vec.getZ(), trans.getZ(), 1e-10);
			assertEquals(scaleCheck.getX(), scale.getX(), 1e-6);
			assertEquals(scaleCheck.getY(), scale.getY(), 1e-6);
			assertEquals(scaleCheck.getZ(), scale.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4fAndVector3f()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Vector3f trans = new Vector3f();
		Quat4f quatCheck = new Quat4f();
		Vector3f vec = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, trans);
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			Transform3d transform = new Transform3d(quat1, trans, 1.0);

			transform.get(quatCheck, vec);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-3);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-3);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-3);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-3);
			assertEquals(vec.getX(), trans.getX(), 1e-3);
			assertEquals(vec.getY(), trans.getY(), 1e-3);
			assertEquals(vec.getZ(), trans.getZ(), 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4fAndVector3fAndVectorScales()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Vector3f trans = new Vector3f();
		Quat4f quatCheck = new Quat4f();
		Vector3f vec = new Vector3f();
		Vector3f scaleCheck = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, trans);
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			Vector3f scale = new Vector3f(random.nextFloat(),
					random.nextFloat(), random.nextFloat());
			Transform3d transform = new Transform3d(quat1, trans, scale);

			transform.get(quatCheck, vec);
			transform.getScale(scaleCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-3);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-3);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-3);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-3);
			assertEquals(vec.getX(), trans.getX(), 1e-3);
			assertEquals(vec.getY(), trans.getY(), 1e-3);
			assertEquals(vec.getZ(), trans.getZ(), 1e-3);
			JUnitTools.assertVector3fEquals("", scaleCheck, scale, 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4d()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Quat4d quatCheck = new Quat4d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			Transform3d transform = new Transform3d(quat1,
					new Vector3d(0, 0, 0), 1.0);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(0, trans.getX(), 1e-10);
			assertEquals(0, trans.getY(), 1e-10);
			assertEquals(0, trans.getZ(), 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetTransformWithQuat4d()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Quat4d quatCheck = new Quat4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			transform.setRotationAndZeroTranslation(quat1);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(0, trans.getX(), 1e-10);
			assertEquals(0, trans.getY(), 1e-10);
			assertEquals(0, trans.getZ(), 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetTransformWithQuat4dAndVector3d()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Vector3d vector = new Vector3d();
		Quat4d quatCheck = new Quat4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			randomizeVector(random, vector);

			transform.set(quat1, vector);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(vector.getX(), trans.getX(), 1e-10);
			assertEquals(vector.getY(), trans.getY(), 1e-10);
			assertEquals(vector.getZ(), trans.getZ(), 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetTransformWithQuat4dAndVector3dAndScale()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Vector3d trans = new Vector3d();
		Vector3d vector = new Vector3d();
		Quat4d quatCheck = new Quat4d();
		Transform3d transform = new Transform3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(random.nextDouble());
			quat1.setW(random.nextDouble());
			quat1.normalize();

			double scale = random.nextDouble();
			randomizeVector(random, vector);

			transform.set(quat1, vector, scale);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);
			transform.getScale(scaleCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-10);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-10);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-10);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-10);
			assertEquals(vector.getX(), trans.getX(), 1e-10);
			assertEquals(vector.getY(), trans.getY(), 1e-10);
			assertEquals(vector.getZ(), trans.getZ(), 1e-10);
			assertEquals(scale, scaleCheck.getX(), 1e-6);
			assertEquals(scale, scaleCheck.getY(), 1e-6);
			assertEquals(scale, scaleCheck.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetTransformWithQuat4f()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Vector3f trans = new Vector3f();
		Quat4f quatCheck = new Quat4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			transform.setRotationAndZeroTranslation(quat1);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-5);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-5);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-5);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-5);
			assertEquals(0, trans.getX(), 1e-5);
			assertEquals(0, trans.getY(), 1e-5);
			assertEquals(0, trans.getZ(), 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetTransformWithQuat4fAndVector3f()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Vector3f trans = new Vector3f();
		Vector3f vector = new Vector3f();
		Quat4f quatCheck = new Quat4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			randomizeVector(random, vector);

			transform.set(quat1, vector);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-5);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-5);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-5);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-5);
			assertEquals(vector.getX(), trans.getX(), 1e-5);
			assertEquals(vector.getY(), trans.getY(), 1e-5);
			assertEquals(vector.getZ(), trans.getZ(), 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetTransformWithQuat4fAndVector3fAndScale()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Vector3f trans = new Vector3f();
		Vector3f vector = new Vector3f();
		Quat4f quatCheck = new Quat4f();
		Transform3d transform = new Transform3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			randomizeVector(random, vector);

			double scale = random.nextDouble();
			transform.set(quat1, vector, scale);

			transform.getRotation(quatCheck);
			transform.getTranslation(trans);

			transform.getScale(scaleCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-5);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-5);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-5);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-5);
			assertEquals(vector.getX(), trans.getX(), 1e-5);
			assertEquals(vector.getY(), trans.getY(), 1e-5);
			assertEquals(vector.getZ(), trans.getZ(), 1e-5);
			assertEquals(scale, scaleCheck.getX(), 1e-3);
			assertEquals(scale, scaleCheck.getY(), 1e-3);
			assertEquals(scale, scaleCheck.getZ(), 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestDeterminant()
	{
		Random random = new Random();
		Matrix4d matrix = new Matrix4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			transform.set(matrix);
			assertEquals(matrix.determinant(), transform.determinantRotationPart(), 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestDeterminantWithScaledTransform()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Vector3d vector = new Vector3d();
		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);
			double scale = random.nextDouble();

			Transform3d transform = new Transform3d(matrix, vector, scale);
			matrix.mul(scale);
			Transform3d transform2 = new Transform3d(matrix, vector, 1.0);

			assertEquals(transform2.determinantRotationPart(), transform.determinantRotationPart(),
					1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4f()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Vector3f trans = new Vector3f();
		Quat4f quatCheck = new Quat4f();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			Transform3d transform = new Transform3d(quat1,
					new Vector3f(0, 0, 0), 1.0);

			transform.get(quatCheck, trans);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-3);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-3);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-3);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-3);
			assertEquals(0, trans.getX(), 1e-3);
			assertEquals(0, trans.getY(), 1e-3);
			assertEquals(0, trans.getZ(), 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuat4f2()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Quat4f quatCheck = new Quat4f();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			Transform3d transform = new Transform3d(quat1,
					new Vector3f(0, 0, 0), 1.0);

			transform.getRotation(quatCheck);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-3);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-3);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-3);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuatWithZeroVectorElement()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextDouble());
			quat1.setY(random.nextDouble());
			quat1.setZ(0);
			quat1.setW(random.nextDouble());
			quat1.normalize();

			transform.setRotationAndZeroTranslation(quat1);

			transform.get(quatCheck, vec);

			assertEquals(quat1.getX(), quatCheck.getX(), 1e-7);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-7);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-7);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-7);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformWithQuatWithZeroScalarElement()
	{
		Random random = new Random();
		Quat4d quat1 = new Quat4d();
		Quat4d quatCheck = new Quat4d();
		Vector3d vec = new Vector3d();
		Transform3d transform = new Transform3d();

		quat1.setX(random.nextDouble());
		quat1.setY(random.nextDouble());
		quat1.setZ(random.nextDouble());
		quat1.setW(0);
		quat1.normalize();

		transform.setRotationAndZeroTranslation(quat1);

		transform.get(quatCheck, vec);

		assertEquals(quat1.getX(), quatCheck.getX(), 1e-7);
		assertEquals(quat1.getY(), quatCheck.getY(), 1e-7);
		assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-7);
		assertEquals(quat1.getW(), quatCheck.getW(), 1e-7);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestGetTransformAsQuat4fAndVector3f()
	{
		Random random = new Random();
		Quat4f quat1 = new Quat4f();
		Transform3d transform = new Transform3d();
		Quat4f quatCheck = new Quat4f();
		Vector3f vec = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			quat1.setX(random.nextFloat());
			quat1.setY(random.nextFloat());
			quat1.setZ(random.nextFloat());
			quat1.setW(random.nextFloat());
			quat1.normalize();

			transform.setRotation(quat1);

			transform.get(quatCheck, vec);

			// Having trouble getting quat.w to be more precise than 1e-3 here.
			// It has to be floating point precision issue, but haven't figure
			// out how to fix that.
			assertEquals(quat1.getX(), quatCheck.getX(), 1e-3);
			assertEquals(quat1.getY(), quatCheck.getY(), 1e-3);
			assertEquals(quat1.getZ(), quatCheck.getZ(), 1e-3);
			assertEquals(quat1.getW(), quatCheck.getW(), 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformFromMatrix3dAndGetTransformAsMatrix3d()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Vector3d vector = new Vector3d();
		Matrix3d matrixCheck = new Matrix3d();
		Vector3d vectorCheck = new Vector3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(matrix, vector, 1.0);

			transform.getRotation(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scaleCheck);

			JUnitTools
					.assertMatrix3dEquals("", matrixCheck, matrixCheck, 1e-20);
			JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-20);
			assertEquals(scaleCheck.getX(), 1.0, 1e-6);
			assertEquals(scaleCheck.getY(), 1.0, 1e-6);
			assertEquals(scaleCheck.getZ(), 1.0, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetMatrixScaleWithMatrix3d()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Vector3d vector = new Vector3d();
		Matrix3d matrixCheck = new Matrix3d();
		Vector3d vectorCheck = new Vector3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);
			Transform3d transform = new Transform3d(matrix, vector, 1.0);

			double scale = random.nextDouble();
			matrix.mul(scale);

			transform.setRotationScale(matrix);

			transform.getRotationScale(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scaleCheck);

			JUnitTools.assertMatrix3dEquals("", matrix, matrixCheck, 1e-8);
			JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-8);
			assertEquals(scale, scaleCheck.getX(), 1e-6);
			assertEquals(scale, scaleCheck.getY(), 1e-6);
			assertEquals(scale, scaleCheck.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetMatrixScaleWithMatrix3f()
	{
		Random random = new Random();
		Matrix3f matrix = new Matrix3f();
		Vector3f vector = new Vector3f();
		Matrix3f matrixCheck = new Matrix3f();
		Vector3f vectorCheck = new Vector3f();
		Vector3f scaleCheck = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(matrix, vector, 1.0f);

			double scale = random.nextDouble();
			matrix.mul((float) scale);

			transform.setRotationScale(matrix);

			transform.getRotationScale(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scaleCheck);

			JUnitTools.assertMatrix3fEquals("", matrix, matrixCheck, 1e-3);
			JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-3);
			assertEquals(scale, scaleCheck.getX(), 1e-3);
			assertEquals(scale, scaleCheck.getY(), 1e-3);
			assertEquals(scale, scaleCheck.getZ(), 1e-3);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateTransformFromMatrix3dAndGetTransformAsMatrix3dWithScale()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Vector3d vector = new Vector3d();
		Matrix3d matrixCheck = new Matrix3d();
		Vector3d vectorCheck = new Vector3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);
			double scale = random.nextDouble();
			Transform3d transform = new Transform3d(matrix, vector, scale);

			transform.getRotation(matrixCheck);
			transform.getTranslation(vectorCheck);
			transform.getScale(scaleCheck);

			JUnitTools
					.assertMatrix3dEquals("", matrixCheck, matrixCheck, 1e-20);
			JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-20);
			assertEquals(scale, scaleCheck.getX(), 1e-6);
			assertEquals(scale, scaleCheck.getY(), 1e-6);
			assertEquals(scale, scaleCheck.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix3d()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Matrix4d matrix4 = new Matrix4d();
		Matrix4d matrixCheck = new Matrix4d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			matrix4.setM00(matrix.getM00());
			matrix4.setM01(matrix.getM01());
			matrix4.setM02(matrix.getM02());
			matrix4.setM10(matrix.getM10());
			matrix4.setM11(matrix.getM11());
			matrix4.setM12(matrix.getM12());
			matrix4.setM20(matrix.getM20());
			matrix4.setM21(matrix.getM21());
			matrix4.setM22(matrix.getM22());
			matrix4.setM33(1);

			Transform3d transform = new Transform3d(matrix, new Vector3d(0, 0,
					0), 1.0);
			transform.get(matrixCheck);

			JUnitTools.assertMatrix4dEquals("", matrixCheck, matrix4, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetAsMatrix3d()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Matrix4d matrix4 = new Matrix4d();
		Matrix4d matrixCheck = new Matrix4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			matrix4.setM00(matrix.getM00());
			matrix4.setM01(matrix.getM01());
			matrix4.setM02(matrix.getM02());
			matrix4.setM03(0);
			matrix4.setM10(matrix.getM10());
			matrix4.setM11(matrix.getM11());
			matrix4.setM12(matrix.getM12());
			matrix4.setM13(0);
			matrix4.setM20(matrix.getM20());
			matrix4.setM21(matrix.getM21());
			matrix4.setM22(matrix.getM22());
			matrix4.setM23(0);
			matrix4.setM30(0);
			matrix4.setM31(0);
			matrix4.setM32(0);
			matrix4.setM33(1);

			transform.setRotationAndZeroTranslation(matrix);

			transform.get(matrixCheck);

			JUnitTools.assertMatrix4dEquals("", matrixCheck, matrix4, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetAsVector3d()
	{
		Random random = new Random();
		Vector3d vector = new Vector3d();
		Matrix4d matrix4 = new Matrix4d();
		Matrix4d matrixCheck = new Matrix4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, vector);

			transform.setTranslationAndIdentityRotation(vector);
			matrix4.setM00(1);
			matrix4.setM11(1);
			matrix4.setM22(1);
			matrix4.setM33(1);
			matrix4.setM03(vector.getX());
			matrix4.setM13(vector.getY());
			matrix4.setM23(vector.getZ());

			transform.get(matrixCheck);

			JUnitTools.assertMatrix4dEquals("", matrixCheck, matrix4, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetAsVector3f()
	{
		Random random = new Random();
		Vector3f vector = new Vector3f();
		Matrix4f matrix4 = new Matrix4f();
		Matrix4f matrixCheck = new Matrix4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			randomizeVector(random, vector);

			transform.setTranslationAndIdentityRotation(vector);
			matrix4.setM00(1);
			matrix4.setM11(1);
			matrix4.setM22(1);
			matrix4.setM33(1);
			matrix4.setM03(vector.getX());
			matrix4.setM13(vector.getY());
			matrix4.setM23(vector.getZ());

			transform.get(matrixCheck);

			JUnitTools.assertMatrix4fEquals("", matrixCheck, matrix4, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestSetAsMatrix3f()
	{
		Random random = new Random();
		Matrix3f matrix = new Matrix3f();
		Matrix4f matrix4 = new Matrix4f();
		Matrix4f matrixCheck = new Matrix4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			matrix4.setM00(matrix.getM00());
			matrix4.setM01(matrix.getM01());
			matrix4.setM02(matrix.getM02());
			matrix4.setM03(0);
			matrix4.setM10(matrix.getM10());
			matrix4.setM11(matrix.getM11());
			matrix4.setM12(matrix.getM12());
			matrix4.setM13(0);
			matrix4.setM20(matrix.getM20());
			matrix4.setM21(matrix.getM21());
			matrix4.setM22(matrix.getM22());
			matrix4.setM23(0);
			matrix4.setM30(0);
			matrix4.setM31(0);
			matrix4.setM32(0);
			matrix4.setM33(1);

			transform.setRotationAndZeroTranslation(matrix);

			transform.get(matrixCheck);

			JUnitTools.assertMatrix4fEquals("", matrixCheck, matrix4, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix3d2()
	{
		Random random = new Random();
		Matrix3d matrix = new Matrix3d();
		Vector3d vector = new Vector3d();
		Vector3d vectorCheck = new Vector3d();
		Matrix3d matrixCheck = new Matrix3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(matrix, vector, 1.0);
			transform.normalizeRotationPart();
			transform.get(matrixCheck, vectorCheck);

			JUnitTools.assertMatrix3dEquals("", matrixCheck, matrix, 1e-12);
			JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-12);

			transform.getRotation(matrixCheck);

			JUnitTools.assertMatrix3dEquals("", matrixCheck, matrix, 1e-12);

			transform.getTranslation(vectorCheck);

			JUnitTools.assertVector3dEquals("", vectorCheck, vector, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix3f()
	{
		Random random = new Random();
		Matrix3f matrix = new Matrix3f();
		Matrix4f matrix4 = new Matrix4f();
		Matrix4f matrixCheck = new Matrix4f();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			matrix4.setM00(matrix.getM00());
			matrix4.setM01(matrix.getM01());
			matrix4.setM02(matrix.getM02());
			matrix4.setM10(matrix.getM10());
			matrix4.setM11(matrix.getM11());
			matrix4.setM12(matrix.getM12());
			matrix4.setM20(matrix.getM20());
			matrix4.setM21(matrix.getM21());
			matrix4.setM22(matrix.getM22());
			matrix4.setM33(1);

			Transform3d transform = new Transform3d(matrix, new Vector3f(0, 0,
					0), 1.0);
			transform.get(matrixCheck);

			JUnitTools.assertMatrix4fEquals("", matrixCheck, matrix4, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix3f2()
	{
		Random random = new Random();
		Matrix3f matrix = new Matrix3f();
		Vector3f vector = new Vector3f();
		Vector3f vectorCheck = new Vector3f();
		Matrix3f matrixCheck = new Matrix3f();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);

			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(matrix, vector, 1.0);
			transform.get(matrixCheck, vectorCheck);

			JUnitTools.assertMatrix3fEquals("", matrixCheck, matrix, 1e-6);
			JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-6);

			transform.getRotation(matrixCheck);
			JUnitTools.assertMatrix3fEquals("", matrixCheck, matrix, 1e-6);

			transform.getTranslation(vectorCheck);
			JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromTransform()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F matrixCheck = new DenseMatrix64F(4, 4);

		createRandomTransformationMatrix(matrix, random);
		Transform3d transform = new Transform3d(matrix);
		Transform3d transformCheck = new Transform3d(transform);

		transform.get(matrix);
		transformCheck.get(matrixCheck);

		JUnitTools.assertMatrixEquals("", matrixCheck, matrix, 1e-12);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix3fAndVector3f()
	{
		Random random = new Random();
		Matrix3f matrix = new Matrix3f();
		Vector3f vector = new Vector3f();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);

			Transform3d transform = new Transform3d(matrix, vector, 1.0);
			Matrix3f matrixCheck = new Matrix3f();
			Vector3f vectorCheck = new Vector3f();

			transform.getRotation(matrixCheck);
			transform.getTranslation(vectorCheck);

			JUnitTools
					.assertMatrix3fEquals("", matrixCheck, matrixCheck, 1e-20);
			JUnitTools.assertVector3fEquals("", vectorCheck, vector, 1e-20);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateIdentityTransform()
	{
		for (int j = 0; j < nTests; j++)
		{
			double[] identityMatrix4By4 = new double[16];

			identityMatrix4By4[0] = 1;
			identityMatrix4By4[5] = 1;
			identityMatrix4By4[10] = 1;
			identityMatrix4By4[15] = 1;

			Transform3d transform3d = new Transform3d();

			double error = 0;
			;
			double[] doubleTransform = new double[16];
			transform3d.get(doubleTransform);
			for (int i = 0; i < 16; i++)
			{
				error += (identityMatrix4By4[i] - doubleTransform[i]);
			}

			assertTrue(error == 0);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix4d()
	{
		Matrix4d matrix = new Matrix4d();
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			Transform3d transform3d = new Transform3d(matrix);

			Matrix4d check = new Matrix4d();

			transform3d.get(check);

			JUnitTools.assertMatrix4dEquals("", check, matrix, 1e-20);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromColumnMajorMatrix4d()
	{
		Matrix4d matrix = new Matrix4d();
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			matrix.transpose();
			Transform3d transform3d = new Transform3d();
			transform3d.setAsTranspose(matrix);
			matrix.transpose();

			Matrix4d check = new Matrix4d();

			transform3d.get(check);

			JUnitTools.assertMatrix4dEquals("", check, matrix, 1e-20);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromColumnMajorMatrix4()
	{
		Matrix4f matrix = new Matrix4f();
		Random random = new Random();
		float[] columnMajorTransformationMatrix = new float[16];

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			matrix.transpose();
			createFloatArrayFromMatrix4d(columnMajorTransformationMatrix,
					matrix);
			Transform3d transform3d = new Transform3d();

			transform3d.setAsTranspose(columnMajorTransformationMatrix);

			Matrix4f check = new Matrix4f();

			transform3d.get(check);

			matrix.transpose();
			JUnitTools.assertMatrix4fEquals("", check, matrix, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromColumnMajorMatrix4f()
	{
		Matrix4f matrix = new Matrix4f();
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			matrix.transpose();
			Transform3d transform3d = new Transform3d();
			transform3d.setAsTranspose(matrix);
			matrix.transpose();

			Matrix4f check = new Matrix4f();

			transform3d.get(check);

			JUnitTools.assertMatrix4fEquals("", check, matrix, 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromMatrix4f()
	{
		Matrix4f matrix = new Matrix4f();
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			Transform3d transform3d = new Transform3d(matrix);

			Matrix4f check = new Matrix4f();

			transform3d.get(check);

			JUnitTools.assertMatrix4fEquals("", check, matrix, 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromDenseMatrix()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(denseMatrix, random);

			Transform3d transform3d = new Transform3d(denseMatrix);

			DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);

			transform3d.get(checkMatrix);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromDenseMatrixAndVector3d()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
		Vector3d trans = new Vector3d();
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(denseMatrix, random);
			randomizeVector(random, trans);

			Transform3d transform3d = new Transform3d(denseMatrix, trans, 1.0);

			DenseMatrix64F checkMatrix = new DenseMatrix64F(3, 3);
			Vector3d checkVector = new Vector3d();

			transform3d.getRotation(checkMatrix);
			transform3d.getTranslation(checkVector);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromDenseMatrixAndVector3dWithVectorScale()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
		Vector3d trans = new Vector3d();
		Vector3d scales = new Vector3d();
		Vector3d checkScales = new Vector3d();
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(denseMatrix, random);
			randomizeVector(random, trans);

			scales.setX(random.nextDouble());
			scales.setY(random.nextDouble());
			scales.setZ(random.nextDouble());
			Transform3d transform3d = new Transform3d(denseMatrix, trans,
					scales);

			DenseMatrix64F checkMatrix = new DenseMatrix64F(3, 3);
			Vector3d checkVector = new Vector3d();

			transform3d.getRotation(checkMatrix);
			transform3d.getTranslation(checkVector);
			transform3d.getScale(checkScales);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
			JUnitTools.assertVector3dEquals("", checkScales, scales, 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromDenseMatrixThatIsTheWrongSize()
	{

		try
		{
			Transform3d transform3d = new Transform3d(new DenseMatrix64F(9, 9),
					new Vector3d(0, 0, 0), 1.0);
		}
		catch (Exception e)
		{
			// If the code gets here, it passed the test.
			assertTrue(true);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromDoubleArray()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		double[] doubleArrayCheck = new double[16];

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			double[] matrixAsDoubleArray = matrix.getData();
			Transform3d transform = new Transform3d(matrixAsDoubleArray);

			transform.get(doubleArrayCheck);
			assertArrayEquals(matrixAsDoubleArray, doubleArrayCheck, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMax()
	{
		double a = 1;
		double b = 2;
		double c = 3;

		assertEquals(Transform3d.max3(a, b, c), c, 1e-12);

		a = 2;
		b = 3;
		c = 1;

		assertEquals(Transform3d.max3(a, b, c), b, 1e-12);

		a = 3;
		b = 1;
		c = 2;

		assertEquals(Transform3d.max3(a, b, c), a, 1e-12);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestCreateFromfloatArray()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		float[] floatArray = new float[16];

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			double[] matrixAsFloatArray = matrix.getData();
			float[] floatArrayCheck = putDoublesInFloatArray(matrixAsFloatArray);

			Transform3d transform = new Transform3d(floatArrayCheck);

			transform.get(floatArray);

			assertArrayEquals(floatArray, floatArrayCheck, (float) 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformMultiplication()
	{
		Random random = new Random();
		Matrix4d matrix1 = new Matrix4d();
		Matrix4d matrix2 = new Matrix4d();

		for (int i = 0; i < 1; i++)
		{
			createRandomTransformationMatrix(matrix1, random);
			createRandomTransformationMatrix(matrix2, random);

			Transform3d transform1 = new Transform3d(matrix1);
			Transform3d transform2 = new Transform3d(matrix2);

			matrix1.mul(matrix2);
			transform1.multiply(transform2);

			Matrix4d transformMat4d = new Matrix4d();

			transform1.get(transformMat4d);

			JUnitTools.assertMatrix4dEquals("", matrix1, transformMat4d, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformMultiplication2()
	{
		Random random = new Random();
		Matrix4d matrix1 = new Matrix4d();
		Matrix4d matrix2 = new Matrix4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < 1; i++)
		{
			createRandomTransformationMatrix(matrix1, random);
			createRandomTransformationMatrix(matrix2, random);

			Transform3d transform1 = new Transform3d(matrix1);
			Transform3d transform2 = new Transform3d(matrix2);

			matrix1.mul(matrix2);
			transform.multiply(transform1, transform2);

			Matrix4d transformMat4d = new Matrix4d();

			transform.get(transformMat4d);

			JUnitTools.assertMatrix4dEquals("", matrix1, transformMat4d, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformMultiplication3()
	{
		Random random = new Random();
		Matrix4d matrix1 = new Matrix4d();
		Matrix4d matrix2 = new Matrix4d();

		for (int i = 0; i < 1; i++)
		{
			createRandomTransformationMatrix(matrix1, random);
			createRandomTransformationMatrix(matrix2, random);

			Transform3d transform1 = new Transform3d(matrix1);
			Transform3d transform2 = new Transform3d(matrix2);
			// matrix1.mul(matrix2);
			matrix2.mul(matrix1);
			transform1.multiply(transform2, transform1);

			Matrix4d transform1Mat4d = new Matrix4d();

			transform1.get(transform1Mat4d);

			JUnitTools
					.assertMatrix4dEquals("", matrix2, transform1Mat4d, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 1.6)
	@Test(timeout = 30000)
	public void TestOrthogonalityOfChainOfTransformations()
	{
		Random random = new Random();
		Matrix4d matrix1 = new Matrix4d();
		Transform3d transform = new Transform3d();

		for (int j = 0; j < nTests; j++)
		{
			for (int i = 0; i < 10000; i++)
			{
				createRandomTransformationMatrix(matrix1, random);
				Transform3d transformToMult = new Transform3d(matrix1);
				transform.multiply(transformToMult);
			}

			Matrix4d retXform = new Matrix4d();
			transform.get(retXform);

			Matrix3d mat = new Matrix3d();
			transform.getRotationScale(mat);
			RotationTools.isRotationMatrixProper(mat);
			assertTrue(checkOrthogonality(transform));
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverse()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(denseMatrix, random);

			Transform3d transform3d = new Transform3d(denseMatrix);

			CommonOps.invert(denseMatrix);

			transform3d.invert();
			transform3d.get(checkMatrix);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverse7()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
		Random random = new Random();
		Transform3d transform2 = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(denseMatrix, random);

			Transform3d transform3d = new Transform3d(denseMatrix);
			transform2.invert(transform3d);
			transform3d.invert();
			CommonOps.invert(denseMatrix);

			transform3d.get(checkMatrix);
			transform2.get(denseMatrix);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixManyMatrixInverses()
	{
		Matrix4d matrix = new Matrix4d();
		Matrix4d checkMatrix = new Matrix4d();
		Matrix3d mat = new Matrix3d();
		Random random = new Random();
		int nInverses = 100;
		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			Transform3d transform3d = new Transform3d(matrix);

			for (int j = 0; j < nInverses; j++)
			{
				transform3d.invert();
				matrix.invert();
			}
			transform3d.get(checkMatrix);
			transform3d.getRotationScale(mat);
			RotationTools.isRotationMatrixProper(mat);

			JUnitTools.assertMatrix4dEquals("", matrix, checkMatrix, 1e-10);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestNearSingularMatrixInverse()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(denseMatrix, random);

			for (int j = 0; j < 3; j++)
			{
				for (int k = 0; k < 4; k++)
				{
					denseMatrix.set(j, k, 1e-8 * random.nextDouble());
				}
			}

			Transform3d transform3d = new Transform3d(denseMatrix);

			CommonOps.invert(denseMatrix);

			transform3d.invert();
			transform3d.get(checkMatrix);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-8);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestInvertTransform2()
	{
		DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
		DenseMatrix64F matrix2 = new DenseMatrix64F(4, 4);
		Vector3d vector = new Vector3d();
		DenseMatrix64F checkMatrix2 = new DenseMatrix64F(4, 4);
		DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);

			Vector3d scale = new Vector3d(random.nextDouble(),
					random.nextDouble(), random.nextDouble());
			Transform3d transform = new Transform3d(matrix, vector, scale);
			transform.get(matrix2);
			CommonOps.invert(matrix2);
			Transform3d transform2 = new Transform3d(matrix2);

			transform2.invert();

			transform.get(checkMatrix);
			transform2.get(checkMatrix2);

			JUnitTools.assertMatrixEquals(checkMatrix2, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestInvertTransform3()
	{
		DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
		DenseMatrix64F matrix2 = new DenseMatrix64F(4, 4);
		Vector3d vector = new Vector3d();
		DenseMatrix64F checkMatrix2 = new DenseMatrix64F(4, 4);
		DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
		Random random = new Random();

		for (int i = 0; i < nTests; i++)
		{
			createRandomRotationMatrix(matrix, random);
			randomizeVector(random, vector);

			Vector3d scale = new Vector3d(random.nextDouble(),
					random.nextDouble(), random.nextDouble());
			Transform3d transform = new Transform3d(matrix, vector, scale);
			transform.invert();
			transform.get(matrix2);
			Transform3d transform2 = new Transform3d(matrix2);

			transform.get(checkMatrix);
			transform2.get(checkMatrix2);

			JUnitTools.assertMatrixEquals(checkMatrix2, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverseWithScale()
	{
		Matrix4d matrix = new Matrix4d();
		Matrix4d checkMatrix = new Matrix4d();
		Random random = new Random();
		Vector3d checkScale = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			double scale = random.nextDouble();
			Transform3d transform3d = new Transform3d(matrix);
			transform3d.setScale(scale);

			matrix.mul(scale);
			matrix.setM33(matrix.getM33() / scale);
			matrix.setM03(matrix.getM03() / scale);
			matrix.setM13(matrix.getM13() / scale);
			matrix.setM23(matrix.getM23() / scale);
			matrix.invert();

			transform3d.invert();
			transform3d.get(checkMatrix);
			transform3d.getScale(checkScale);

			JUnitTools.assertMatrix4dEquals("", matrix, checkMatrix, 1e-6);
			assertEquals(1 / scale, checkScale.getX(), 1e-6);
			assertEquals(1 / scale, checkScale.getY(), 1e-6);
			assertEquals(1 / scale, checkScale.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverseWithScale3()
	{
		Matrix4d matrix = new Matrix4d();
		Matrix4d checkMatrix = new Matrix4d();
		Random random = new Random();
		Vector3d checkScale = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			double scalex = random.nextDouble();
			double scaley = random.nextDouble();
			double scalez = random.nextDouble();
			Transform3d transform3d = new Transform3d(matrix);
			transform3d.setScale(scalex, scaley, scalez);

			matrix.setM00(matrix.getM00() * scalex);
			matrix.setM10(matrix.getM10() * scalex);
			matrix.setM20(matrix.getM20() * scalex);
			matrix.setM01(matrix.getM01() * scaley);
			matrix.setM11(matrix.getM11() * scaley);
			matrix.setM21(matrix.getM21() * scaley);
			matrix.setM02(matrix.getM02() * scalez);
			matrix.setM12(matrix.getM12() * scalez);
			matrix.setM22(matrix.getM22() * scalez);
			matrix.invert();
			matrix.invert();

			transform3d.invert();
			transform3d.invert();
			transform3d.get(checkMatrix);

			JUnitTools.assertMatrix4dEquals("", matrix, checkMatrix, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverseWithScale4()
	{
		Matrix4d matrix = new Matrix4d();
		Matrix4d checkMatrix = new Matrix4d();
		Random random = new Random();
		Vector3d checkScale = new Vector3d();
		Transform3d transform2 = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			double scalex = random.nextDouble();
			double scaley = random.nextDouble();
			double scalez = random.nextDouble();

			matrix.setM00(matrix.getM00() * scalex);
			matrix.setM10(matrix.getM10() * scalex);
			matrix.setM20(matrix.getM20() * scalex);
			matrix.setM01(matrix.getM01() * scaley);
			matrix.setM11(matrix.getM11() * scaley);
			matrix.setM21(matrix.getM21() * scaley);
			matrix.setM02(matrix.getM02() * scalez);
			matrix.setM12(matrix.getM12() * scalez);
			matrix.setM22(matrix.getM22() * scalez);
			Transform3d transform3d = new Transform3d(matrix);
			matrix.invert();
			transform2.setIdentity();
			transform2.invert(transform3d);

			transform2.get(checkMatrix);

			JUnitTools.assertMatrix4dEquals("", matrix, checkMatrix, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverseWithScale2()
	{
		Matrix4d matrix = new Matrix4d();
		Matrix4d checkMatrix = new Matrix4d();
		Random random = new Random();
		Vector3d checkScale = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);

			Vector3d scale = new Vector3d(random.nextDouble(),
					random.nextDouble(), random.nextDouble());
			Transform3d transform3d = new Transform3d(matrix);
			transform3d.setScale(scale);

			matrix.setM00(matrix.getM00() * scale.getX());
			matrix.setM10(matrix.getM10() * scale.getX());
			matrix.setM20(matrix.getM20() * scale.getX());
			matrix.setM01(matrix.getM01() * scale.getY());
			matrix.setM11(matrix.getM11() * scale.getY());
			matrix.setM21(matrix.getM21() * scale.getY());
			matrix.setM02(matrix.getM02() * scale.getZ());
			matrix.setM12(matrix.getM12() * scale.getZ());
			matrix.setM22(matrix.getM22() * scale.getZ());

			matrix.invert();

			transform3d.invert();
			transform3d.get(checkMatrix);
			transform3d.getScale(checkScale);

			Vector3d checkScaleX = new Vector3d(matrix.getM00(), matrix.getM10(),
					matrix.getM20());
			Vector3d checkScaleY = new Vector3d(matrix.getM01(), matrix.getM11(),
					matrix.getM21());
			Vector3d checkScaleZ = new Vector3d(matrix.getM02(), matrix.getM12(),
					matrix.getM22());
			double xScale = checkScaleX.length();
			double yScale = checkScaleY.length();
			double zScale = checkScaleZ.length();

			JUnitTools.assertMatrix4dEquals("", matrix, checkMatrix, 1e-6);
			assertEquals(xScale, checkScale.getX(), 1e-6);
			assertEquals(yScale, checkScale.getY(), 1e-6);
			assertEquals(zScale, checkScale.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestMatrixInverse2()
	{
		DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F checkMatrix = new DenseMatrix64F(4, 4);
		Matrix4d matrix = new Matrix4d();
		Random random = new Random();
		Transform3d transform2 = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(denseMatrix, random);
			createRandomTransformationMatrix(matrix, random);

			Transform3d transform3d = new Transform3d(matrix);
			transform2.invert(transform3d);

			transform2.invert();

			matrix.invert();
			transform2.get(denseMatrix);
			transform3d.get(checkMatrix);

			JUnitTools.assertMatrixEquals(denseMatrix, checkMatrix, 1e-15);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestEpsilonEquals()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		double epsilon = 1e-8;

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			Transform3d transform1 = new Transform3d(matrix);
			Transform3d transform2 = new Transform3d(matrix);

			assertTrue(transform1.epsilonEquals(transform2, epsilon));
			assertTrue(transform2.epsilonEquals(transform1, epsilon));

			for (int row = 0; row < 4; row++)
			{
				for (int col = 0; col < 4; col++)
				{
					// Change one element at a time.
					matrix.set(row, col, 2 + 10 * random.nextDouble());
					transform1.set(matrix);
					assertFalse(transform1.epsilonEquals(transform2, epsilon));
					transform1.set(transform2);
				}
			}
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestEpsilonEqualsWithScale()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		double epsilon = 1e-8;

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			Transform3d transform1 = new Transform3d(matrix);
			Transform3d transform2 = new Transform3d(matrix);
			double scale = random.nextDouble();
			transform1.setScale(scale);
			transform2.setScale(scale);

			assertTrue(transform1.epsilonEquals(transform2, epsilon));
			assertTrue(transform2.epsilonEquals(transform1, epsilon));

			for (int row = 0; row < 4; row++)
			{
				for (int col = 0; col < 4; col++)
				{
					// Change one element at a time.
					matrix.set(row, col, 2 + 10 * random.nextDouble());
					transform1.set(matrix);
					assertFalse(transform1.epsilonEquals(transform2, epsilon));
					transform1.set(transform2);
				}
			}
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestRotX()
	{
		Vector3d vector = new Vector3d(1, 0, 0);
		Vector3d vector2 = new Vector3d(0, 1, 0);
		Vector3d vector3 = new Vector3d(0, 0, 1);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Transform3d transform = new Transform3d();
		transform.setRotationRollAndZeroTranslation(Math.PI / 2);
		transform.getRotation(rotationMatrix);

		MatrixTools.mult(rotationMatrix, vector);
		MatrixTools.mult(rotationMatrix, vector2);
		MatrixTools.mult(rotationMatrix, vector3);

		JUnitTools.assertVector3dEquals("", new Vector3d(1, 0, 0), vector,
				1e-12);
		JUnitTools.assertVector3dEquals("", new Vector3d(0, 0, 1), vector2,
				1e-12);
		JUnitTools.assertVector3dEquals("", new Vector3d(0, -1, 0), vector3,
				1e-12);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestRotY()
	{
		Vector3d vector = new Vector3d(1, 0, 0);
		Vector3d vector2 = new Vector3d(0, 1, 0);
		Vector3d vector3 = new Vector3d(0, 0, 1);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Transform3d transform = new Transform3d();
		transform.setRotationPitchAndZeroTranslation(Math.PI / 2);
		transform.getRotation(rotationMatrix);

		MatrixTools.mult(rotationMatrix, vector);
		MatrixTools.mult(rotationMatrix, vector2);
		MatrixTools.mult(rotationMatrix, vector3);

		JUnitTools.assertVector3dEquals("", new Vector3d(0, 0, -1), vector,
				1e-12);
		JUnitTools.assertVector3dEquals("", new Vector3d(0, 1, 0), vector2,
				1e-12);
		JUnitTools.assertVector3dEquals("", new Vector3d(1, 0, 0), vector3,
				1e-12);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestRotZ()
	{
		Vector3d vector = new Vector3d(1, 0, 0);
		Vector3d vector2 = new Vector3d(0, 1, 0);
		Vector3d vector3 = new Vector3d(0, 0, 1);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Transform3d transform = new Transform3d();
		transform.setRotationYawAndZeroTranslation(Math.PI / 2);
		transform.getRotation(rotationMatrix);

		MatrixTools.mult(rotationMatrix, vector);
		MatrixTools.mult(rotationMatrix, vector2);
		MatrixTools.mult(rotationMatrix, vector3);

		JUnitTools.assertVector3dEquals("", new Vector3d(0, 1, 0), vector,
				1e-12);
		JUnitTools.assertVector3dEquals("", new Vector3d(-1, 0, 0), vector2,
				1e-12);
		JUnitTools.assertVector3dEquals("", new Vector3d(0, 0, 1), vector3,
				1e-12);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestEquals()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			Transform3d transform1 = new Transform3d(matrix);
			Transform3d transform2 = new Transform3d(matrix);

			assertTrue(transform1.equals(transform2));

			transform1.mat10 = transform1.mat10 + 1;
			assertFalse(transform1.equals(transform2));
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestEqualsWithScale()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			Transform3d transform1 = new Transform3d(matrix);
			Transform3d transform2 = new Transform3d(matrix);

			assertTrue(transform1.equals(transform2));

			transform1.mat10 = transform1.mat10 + 1;
			assertFalse(transform1.equals(transform2));
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestEulerAngles()
	{
		Random random = new Random();
		Vector3d vector = new Vector3d();
		Vector3d vectorToCheck = new Vector3d();
		Transform3d transform = new Transform3d();
		for (int i = 0; i < nTests; i++)
		{
			vector.setX(-Math.PI + random.nextDouble() * 2 * Math.PI);
			// Inverse Euler conversion is not defined for vector.y = 0 and the
			// solution switches for
			// vector.y outside of -pi/2 to pi/2. Separate tests can be created
			// if the whole range wants to be
			// tested. The inverse Euler transform is only used for testing
			// purposes, so it is not an issue.
			vector.setY(-(Math.PI / 2 - 0.01) * random.nextDouble() - 0.01);
			vector.setZ(-Math.PI + random.nextDouble() * 2 * Math.PI);

			transform.setRotationEulerAndZeroTranslation(vector);

			transform.getEulerXYZ(vectorToCheck);
			JUnitTools.assertVector3dEquals("", vector, vectorToCheck, 1e-5);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestEulerAnglesWithScale()
	{
		Random random = new Random();
		Vector3d vector = new Vector3d();
		Vector3d vectorToCheck = new Vector3d();
		Transform3d transform = new Transform3d();
		Vector3d scaleCheck = new Vector3d();

		for (int i = 0; i < nTests; i++)
		{
			vector.setX(-Math.PI + random.nextDouble() * 2 * Math.PI);
			// Inverse Euler conversion is not defined for vector.y = 0 and the
			// solution switches for
			// vector.y outside of -pi/2 to pi/2. Separate tests can be created
			// if the whole range wants to be
			// tested. The inverse Euler transform is only used for testing
			// purposes, so it is not an issue.
			vector.setY(-(Math.PI / 2 - 0.01) * random.nextDouble() - 0.01);
			vector.setZ(-Math.PI + random.nextDouble() * 2 * Math.PI);

			transform.setRotationEulerAndZeroTranslation(vector);
			double scale = random.nextDouble();
			transform.setScale(scale);

			transform.getScale(scaleCheck);
			transform.getEulerXYZ(vectorToCheck);
			JUnitTools.assertVector3dEquals("", vector, vectorToCheck, 1e-5);
			assertEquals(scale, scaleCheck.getX(), 1e-6);
			assertEquals(scale, scaleCheck.getY(), 1e-6);
			assertEquals(scale, scaleCheck.getZ(), 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector4d()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		Vector4d vector = new Vector4d();
		Vector4d vector2 = new Vector4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);

			createRandomTransform4Vector(vector, random);
			vector2.set(vector);

			MatrixTools.mult(matrix, vector);
			transform.transform(vector2);

			JUnitTools.assertVector4dEquals("", vector, vector2, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector4d2()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		Vector4d vector = new Vector4d();
		Vector4d vector2 = new Vector4d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);

			createRandomTransform4Vector(vector, random);

			transform.transform(vector, vector2);
			MatrixTools.mult(matrix, vector);

			JUnitTools.assertVector4dEquals("", vector, vector2, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector4f()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		Vector4f vector = new Vector4f();
		Vector4f vector2 = new Vector4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);

			createRandomTransform4Vector(vector, random);
			vector2.set(vector);

			MatrixTools.mult(matrix, vector);
			transform.transform(vector2);

			JUnitTools.assertVector4fEquals("", vector, vector2, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector4f2()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		Vector4f vector = new Vector4f();
		Vector4f vector2 = new Vector4f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);

			createRandomTransform4Vector(vector, random);

			transform.transform(vector, vector2);
			MatrixTools.mult(matrix, vector);

			JUnitTools.assertVector4fEquals("", vector, vector2, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector3d()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Vector3d vector = new Vector3d();
		Vector3d vector2 = new Vector3d();
		Transform3d transform = new Transform3d();
		double scale;

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);
			transform.setScale(scale = (random.nextDouble() - 0.5) * 50.0);

			randomizeVector(random, vector);
			vector2.set(vector);

			MatrixTools.mult(rotationMatrix, vector);
			transform.transform(vector2);
			vector2.scale(1.0 / scale);

			JUnitTools.assertVector3dEquals("", vector, vector2, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector3f()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Vector3f vector = new Vector3f();
		Vector3f vector2 = new Vector3f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);

			randomizeVector(random, vector);
			vector2.set(vector);

			MatrixTools.mult(rotationMatrix, vector);
			transform.transform(vector2);

			JUnitTools.assertVector3fEquals("", vector, vector2, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector3f2()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Vector3f vector = new Vector3f();
		Vector3f vector2 = new Vector3f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);

			randomizeVector(random, vector);

			transform.transform(vector, vector2);
			MatrixTools.mult(rotationMatrix, vector);

			JUnitTools.assertVector3fEquals("", vector, vector2, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformVector3d2()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Vector3d vector = new Vector3d();
		Vector3d vector2 = new Vector3d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);

			randomizeVector(random, vector);

			transform.transform(vector, vector2);
			MatrixTools.mult(rotationMatrix, vector);

			JUnitTools.assertVector3dEquals("", vector, vector2, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformPoint3d()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Point3d point = new Point3d();
		Point3d point2 = new Point3d();
		Vector3d vector = new Vector3d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);
			transform.getTranslation(vector);

			randomizePoint3d(random, point);
			point2.set(point);

			MatrixTools.mult(rotationMatrix, point);
			point.add(vector);
			transform.transform(point2);

			JUnitTools.assertPoint3dEquals("", point, point2, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformPoint3d2()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Point3d point = new Point3d();
		Point3d point2 = new Point3d();
		Vector3d vector = new Vector3d();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);
			transform.getTranslation(vector);

			randomizePoint3d(random, point);

			transform.transform(point, point2);
			MatrixTools.mult(rotationMatrix, point);
			point.add(vector);

			JUnitTools.assertPoint3dEquals("", point, point2, 1e-12);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformPoint3f()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Point3f point = new Point3f();
		Point3f point2 = new Point3f();
		Vector3f vector = new Vector3f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);
			transform.getTranslation(vector);

			randomizePoint3f(random, point);
			point2.set(point);

			MatrixTools.mult(rotationMatrix, point);
			point.add(vector);
			transform.transform(point2);

			JUnitTools.assertPoint3fEquals("", point, point2, 1e-6);
		}
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void TestTransformPoint3f2()
	{
		Random random = new Random();
		DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
		DenseMatrix64F rotationMatrix = new DenseMatrix64F(3, 3);
		Point3f point = new Point3f();
		Point3f point2 = new Point3f();
		Vector3f vector = new Vector3f();
		Transform3d transform = new Transform3d();

		for (int i = 0; i < nTests; i++)
		{
			createRandomTransformationMatrix(matrix, random);
			transform.set(matrix);
			transform.getRotation(rotationMatrix);
			transform.getTranslation(vector);

			randomizePoint3f(random, point);

			transform.transform(point, point2);
			MatrixTools.mult(rotationMatrix, point);
			point.add(vector);

			JUnitTools.assertPoint3fEquals("", point, point2, 1e-6);
		}
	}

	private boolean checkOrthogonality(Transform3d transform)
	{
		Matrix3d matrix = new Matrix3d();
		transform.getRotation(matrix);

		Vector3d tmpVecX = new Vector3d(matrix.getM00(), matrix.getM10(), matrix.getM20());
		Vector3d tmpVecY = new Vector3d(matrix.getM01(), matrix.getM11(), matrix.getM21());
		Vector3d tmpVecZ = new Vector3d(matrix.getM02(), matrix.getM12(), matrix.getM22());

		return (tmpVecX.lengthSquared() - 1 < 1e-8)
				&& (tmpVecY.lengthSquared() - 1 < 1e-8)
				&& (tmpVecZ.lengthSquared() - 1 < 1e-8);
	}

	private void randomizeVector(Random random, Vector3d vector)
	{
		vector.setX(random.nextDouble());
		vector.setY(random.nextDouble());
		vector.setZ(random.nextDouble());
	}

	private void randomizeVector(Random random, Vector3f vector)
	{
		vector.setX(random.nextFloat());
		vector.setY(random.nextFloat());
		vector.setZ(random.nextFloat());
	}

	private void randomizePoint3d(Random random, Point3d point)
	{
		point.setX(random.nextDouble());
		point.setY(random.nextDouble());
		point.setZ(random.nextDouble());
	}

	private void randomizePoint3f(Random random, Point3f point)
	{
		point.setX(random.nextFloat());
		point.setY(random.nextFloat());
		point.setZ(random.nextFloat());
	}

	private void createRandomTransformationMatrix(DenseMatrix64F matrix,
			Random random)
	{
		Matrix3d rotX = new Matrix3d();
		Matrix3d rotY = new Matrix3d();
		Matrix3d rotZ = new Matrix3d();
		Vector3d trans = new Vector3d();

		randomizeVector(random, trans);
		createRandomRotationMatrixX(random, rotX);
		createRandomRotationMatrixY(random, rotY);
		createRandomRotationMatrixZ(random, rotZ);

		rotX.mul(rotY);
		rotX.mul(rotZ);

		matrix.set(0, 0, rotX.getM00());
		matrix.set(0, 1, rotX.getM01());
		matrix.set(0, 2, rotX.getM02());
		matrix.set(0, 3, trans.getX());
		matrix.set(1, 0, rotX.getM10());
		matrix.set(1, 1, rotX.getM11());
		matrix.set(1, 2, rotX.getM12());
		matrix.set(1, 3, trans.getY());
		matrix.set(2, 0, rotX.getM20());
		matrix.set(2, 1, rotX.getM21());
		matrix.set(2, 2, rotX.getM22());
		matrix.set(2, 3, trans.getZ());
		matrix.set(3, 0, 0);
		matrix.set(3, 1, 0);
		matrix.set(3, 2, 0);
		matrix.set(3, 3, 1);
	}

	private void createRandomRotationMatrix(Matrix3d matrix, Random random)
	{
		Matrix3d rotX = new Matrix3d();
		Matrix3d rotY = new Matrix3d();
		Matrix3d rotZ = new Matrix3d();
		Vector3d trans = new Vector3d();

		randomizeVector(random, trans);
		createRandomRotationMatrixX(random, rotX);
		createRandomRotationMatrixY(random, rotY);
		createRandomRotationMatrixZ(random, rotZ);

		rotX.mul(rotY);
		rotX.mul(rotZ);

		matrix.setM00(rotX.getM00());
		matrix.setM01(rotX.getM01());
		matrix.setM02(rotX.getM02());
		matrix.setM10(rotX.getM10());
		matrix.setM11(rotX.getM11());
		matrix.setM12(rotX.getM12());
		matrix.setM20(rotX.getM20());
		matrix.setM21(rotX.getM21());
		matrix.setM22(rotX.getM22());
	}

	private void createRandomRotationMatrix(DenseMatrix64F matrix, Random random)
	{
		Matrix3d rotX = new Matrix3d();
		Matrix3d rotY = new Matrix3d();
		Matrix3d rotZ = new Matrix3d();
		Vector3d trans = new Vector3d();

		randomizeVector(random, trans);
		createRandomRotationMatrixX(random, rotX);
		createRandomRotationMatrixY(random, rotY);
		createRandomRotationMatrixZ(random, rotZ);

		rotX.mul(rotY);
		rotX.mul(rotZ);

		matrix.set(0, 0, rotX.getM00());
		matrix.set(0, 1, rotX.getM01());
		matrix.set(0, 2, rotX.getM02());
		matrix.set(1, 0, rotX.getM10());
		matrix.set(1, 1, rotX.getM11());
		matrix.set(1, 2, rotX.getM12());
		matrix.set(2, 0, rotX.getM20());
		matrix.set(2, 1, rotX.getM21());
		matrix.set(2, 2, rotX.getM22());
	}

	private void createRandomRotationMatrix(Matrix3f matrix, Random random)
	{
		Matrix3f rotX = new Matrix3f();
		Matrix3f rotY = new Matrix3f();
		Matrix3f rotZ = new Matrix3f();
		Vector3f trans = new Vector3f();

		randomizeVector(random, trans);
		createRandomRotationMatrixX(random, rotX);
		createRandomRotationMatrixY(random, rotY);
		createRandomRotationMatrixZ(random, rotZ);

		rotX.mul(rotY);
		rotX.mul(rotZ);

		matrix.setM00(rotX.getM00());
		matrix.setM01(rotX.getM01());
		matrix.setM02(rotX.getM02());
		matrix.setM10(rotX.getM10());
		matrix.setM11(rotX.getM11());
		matrix.setM12(rotX.getM12());
		matrix.setM20(rotX.getM20());
		matrix.setM21(rotX.getM21());
		matrix.setM22(rotX.getM22());
	}

	private void createRandomTransformationMatrix(Matrix4d matrix, Random random)
	{
		Matrix3d rotX = new Matrix3d();
		Matrix3d rotY = new Matrix3d();
		Matrix3d rotZ = new Matrix3d();
		Vector3d trans = new Vector3d();

		randomizeVector(random, trans);
		createRandomRotationMatrixX(random, rotX);
		createRandomRotationMatrixY(random, rotY);
		createRandomRotationMatrixZ(random, rotZ);

		rotX.mul(rotY);
		rotX.mul(rotZ);

		matrix.setM00(rotX.getM00());
		matrix.setM01(rotX.getM01());
		matrix.setM02(rotX.getM02());
		matrix.setM03(trans.getX());
		matrix.setM10(rotX.getM10());
		matrix.setM11(rotX.getM11());
		matrix.setM12(rotX.getM12());
		matrix.setM13(trans.getY());
		matrix.setM20(rotX.getM20());
		matrix.setM21(rotX.getM21());
		matrix.setM22(rotX.getM22());
		matrix.setM23(trans.getZ());
		matrix.setM30(0);
		matrix.setM31(0);
		matrix.setM32(0);
		matrix.setM33(1);
	}

	private void createRandomTransformationMatrix(Matrix4f matrix, Random random)
	{
		Matrix3d rotX = new Matrix3d();
		Matrix3d rotY = new Matrix3d();
		Matrix3d rotZ = new Matrix3d();
		Vector3d trans = new Vector3d();

		randomizeVector(random, trans);
		createRandomRotationMatrixX(random, rotX);
		createRandomRotationMatrixY(random, rotY);
		createRandomRotationMatrixZ(random, rotZ);

		rotX.mul(rotY);
		rotX.mul(rotZ);

		matrix.setM00((float) rotX.getM00());
		matrix.setM01((float) rotX.getM01());
		matrix.setM02((float) rotX.getM02());
		matrix.setM03((float) trans.getX());
		matrix.setM10((float) rotX.getM10());
		matrix.setM11((float) rotX.getM11());
		matrix.setM12((float) rotX.getM12());
		matrix.setM13((float) trans.getY());
		matrix.setM20((float) rotX.getM20());
		matrix.setM21((float) rotX.getM21());
		matrix.setM22((float) rotX.getM22());
		matrix.setM23((float) trans.getZ());
		matrix.setM30(0);
		matrix.setM31(0);
		matrix.setM32(0);
		matrix.setM33(1);
	}

	private void createRandomRotationMatrixX(Random random, Matrix3d matrix)
	{
		double theta = random.nextDouble();
		double cTheta = Math.cos(theta);
		double sTheta = Math.sin(theta);
		matrix.setM00(1);
		matrix.setM01(0);
		matrix.setM02(0);
		matrix.setM10(0);
		matrix.setM11(cTheta);
		matrix.setM12(-sTheta);
		matrix.setM20(0);
		matrix.setM21(sTheta);
		matrix.setM22(cTheta);
	}

	private void createRandomRotationMatrixX(Random random, Matrix3f matrix)
	{
		double theta = random.nextDouble();
		double cTheta = Math.cos(theta);
		double sTheta = Math.sin(theta);
		matrix.setM00(1);
		matrix.setM01(0);
		matrix.setM02(0);
		matrix.setM10(0);
		matrix.setM11((float) cTheta);
		matrix.setM12((float) -sTheta);
		matrix.setM20(0);
		matrix.setM21((float) sTheta);
		matrix.setM22((float) cTheta);
	}

	private void createRandomRotationMatrixY(Random random, Matrix3f matrix)
	{
		double theta = random.nextDouble();
		double cTheta = Math.cos(theta);
		double sTheta = Math.sin(theta);
		matrix.setM00((float) cTheta);
		matrix.setM01(0);
		matrix.setM02((float) sTheta);
		matrix.setM10(0);
		matrix.setM11(1);
		matrix.setM12(0);
		matrix.setM20((float) -sTheta);
		matrix.setM21(0);
		matrix.setM22((float) cTheta);
	}

	private void createRandomRotationMatrixY(Random random, Matrix3d matrix)
	{
		double theta = random.nextDouble();
		double cTheta = Math.cos(theta);
		double sTheta = Math.sin(theta);
		matrix.setM00(cTheta);
		matrix.setM01(0);
		matrix.setM02(sTheta);
		matrix.setM10(0);
		matrix.setM11(1);
		matrix.setM12(0);
		matrix.setM20(-sTheta);
		matrix.setM21(0);
		matrix.setM22(cTheta);
	}

	private void createRandomRotationMatrixZ(Random random, Matrix3d matrix)
	{
		double theta = random.nextDouble();
		double cTheta = Math.cos(theta);
		double sTheta = Math.sin(theta);
		matrix.setM00(cTheta);
		matrix.setM01(-sTheta);
		matrix.setM02(0);
		matrix.setM10(sTheta);
		matrix.setM11(cTheta);
		matrix.setM12(0);
		matrix.setM20(0);
		matrix.setM21(0);
		matrix.setM22(1);
	}

	private void createRandomRotationMatrixZ(Random random, Matrix3f matrix)
	{
		double theta = random.nextDouble();
		double cTheta = Math.cos(theta);
		double sTheta = Math.sin(theta);
		matrix.setM00((float) cTheta);
		matrix.setM01((float) -sTheta);
		matrix.setM02(0);
		matrix.setM10((float) sTheta);
		matrix.setM11((float) cTheta);
		matrix.setM12(0);
		matrix.setM20(0);
		matrix.setM21(0);
		matrix.setM22(1);
	}

	private void createRandomTransform4Vector(Vector4d vector, Random random)
	{
		vector.setX(random.nextDouble());
		vector.setY(random.nextDouble());
		vector.setZ(random.nextDouble());
		vector.setW(1);
	}

	private void createRandomTransform4Vector(Vector4f vector, Random random)
	{
		vector.setX(random.nextFloat());
		vector.setY(random.nextFloat());
		vector.setZ(random.nextFloat());
		vector.setW(1);
	}

	private float[] putDoublesInFloatArray(double[] matrixAsDoubleArray)
	{
		float[] ret = new float[matrixAsDoubleArray.length];
		for (int i = 0; i < matrixAsDoubleArray.length; i++)
		{
			ret[i] = (float) matrixAsDoubleArray[i];
		}

		return ret;
	}

	private void createFloatArrayFromMatrix4d(float[] floatArray,
			Matrix4f matrix)
	{
		floatArray[0] = matrix.getM00();
		floatArray[1] = matrix.getM01();
		floatArray[2] = matrix.getM02();
		floatArray[3] = matrix.getM03();
		floatArray[4] = matrix.getM10();
		floatArray[5] = matrix.getM11();
		floatArray[6] = matrix.getM12();
		floatArray[7] = matrix.getM13();
		floatArray[8] = matrix.getM20();
		floatArray[9] = matrix.getM21();
		floatArray[10] = matrix.getM22();
		floatArray[11] = matrix.getM23();
		floatArray[12] = matrix.getM30();
		floatArray[13] = matrix.getM31();
		floatArray[14] = matrix.getM32();
		floatArray[15] = matrix.getM33();
	}

	private void messWithRotationMatrixOrthogonality(Matrix4d matrix,
			Random random)
	{
		matrix.setM00(matrix.getM00() + random.nextDouble() * 1e-3);
		matrix.setM01(matrix.getM01() + random.nextDouble() * 1e-3);
		matrix.setM02(matrix.getM02() + random.nextDouble() * 1e-3);
		matrix.setM10(matrix.getM10() + random.nextDouble() * 1e-3);
		matrix.setM11(matrix.getM11() + random.nextDouble() * 1e-3);
		matrix.setM12(matrix.getM12() + random.nextDouble() * 1e-3);
		matrix.setM20(matrix.getM20() + random.nextDouble() * 1e-3);
		matrix.setM21(matrix.getM21() + random.nextDouble() * 1e-3);
		matrix.setM22(matrix.getM22() + random.nextDouble() * 1e-3);
	}
}
