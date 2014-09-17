package com.yobotics.simulationconstructionset.physics.collision.gdx;

import static org.junit.Assert.assertEquals;

import us.ihmc.utilities.math.geometry.Transform3d;

import org.junit.Test;

import com.badlogic.gdx.math.Matrix4;

/**
 * @author Peter Abeles
 */
public class GdxUtilTest
{
	@Test
	public void convert_t2m()
	{
		float[] m = new float[]{
				1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16};

		Transform3d src = new Transform3d();
		Matrix4 dst = new Matrix4();

		src.set(m);

		GdxUtil.convert(src,dst);

		float[] a= dst.getValues();

		for( int y = 0; y < 4; y++ ) {
			for( int x = 0; x < 4; x++ )  {
				int index = x*4 + y;
				int expected = y*4 + x + 1;
				assertEquals(expected,a[index],1e-6f);
			}
		}
	}

	@Test
	public void convert_m2t()
	{
		float[] m = new float[]{
				1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16};

		Matrix4 src = new Matrix4();
		Transform3d dst = new Transform3d();

		src.set(m);

		GdxUtil.convert(src,dst);

		float[] a = new float[16];
		dst.get(a);

		for( int y = 0; y < 4; y++ ) {
			for( int x = 0; x < 4; x++ )  {
				int index = x*4 + y;
				int expected = y*4 + x + 1;
				assertEquals(expected,a[index],1e-6f);
			}
		}
	}
}
