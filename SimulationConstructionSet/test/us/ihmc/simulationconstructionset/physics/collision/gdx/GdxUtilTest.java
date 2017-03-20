package us.ihmc.simulationconstructionset.physics.collision.gdx;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import com.badlogic.gdx.math.Matrix4;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * @author Peter Abeles
 */
public class GdxUtilTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void convert_t2m()
	{
      Random random = new Random(234234L);
      RotationMatrix randomRotation = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
		float[] m = new float[]{
				(float) randomRotation.getM00(), (float) randomRotation.getM01(), (float) randomRotation.getM02(), 4,
				(float) randomRotation.getM10(), (float) randomRotation.getM11(), (float) randomRotation.getM12(), 8,
				(float) randomRotation.getM20(), (float) randomRotation.getM21(), (float) randomRotation.getM22(), 12,
				0, 0, 0, 1};

		RigidBodyTransform src = new RigidBodyTransform();
		Matrix4 dst = new Matrix4();

		src.set(m);

		GdxUtil.convert(src,dst);

		float[] a= dst.getValues();

		assertEquals(m[0],a[0],1e-6f);
      assertEquals(m[4],a[1],1e-6f);
      assertEquals(m[8],a[2],1e-6f);
      assertEquals(m[12],a[3],1e-6f);
      assertEquals(m[1],a[4],1e-6f);
      assertEquals(m[5],a[5],1e-6f);
      assertEquals(m[9],a[6],1e-6f);
      assertEquals(m[13],a[7],1e-6f);
      assertEquals(m[10],a[10],1e-6f);
      assertEquals(m[15],a[15],1e-6f);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void convert_m2t()
	{
	   Random random = new Random(234234L);
      RotationMatrix randomRotation = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
		float[] m = new float[]{
		      (float) randomRotation.getM00(), (float) randomRotation.getM01(), (float) randomRotation.getM02(), 0,
		      (float) randomRotation.getM10(), (float) randomRotation.getM11(), (float) randomRotation.getM12(), 0,
		      (float) randomRotation.getM20(), (float) randomRotation.getM21(), (float) randomRotation.getM22(), 0,
				13, 14, 15, 1};

		Matrix4 src = new Matrix4();
		RigidBodyTransform dst = new RigidBodyTransform();

		src.set(m);

		GdxUtil.convert(src,dst);

		float[] a = new float[16];
		dst.get(a);
		assertEquals(m[0],a[0],1e-6f);
		assertEquals(m[4],a[1],1e-6f);
		assertEquals(m[8],a[2],1e-6f);
		assertEquals(m[12],a[3],1e-6f);
		assertEquals(m[1],a[4],1e-6f);
		assertEquals(m[5],a[5],1e-6f);
		assertEquals(m[9],a[6],1e-6f);
		assertEquals(m[13],a[7],1e-6f);
		assertEquals(m[10],a[10],1e-6f);
		assertEquals(m[15],a[15],1e-6f);
	}
}
