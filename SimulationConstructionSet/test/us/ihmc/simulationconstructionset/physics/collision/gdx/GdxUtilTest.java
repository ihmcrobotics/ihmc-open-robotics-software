package us.ihmc.simulationconstructionset.physics.collision.gdx;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import com.badlogic.gdx.math.Matrix4;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
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
		float[] m = new float[]{
				1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
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
		float[] m = new float[]{
				1, 2, 3, 0,
				5, 6, 7, 0,
				9, 10, 11, 0,
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
