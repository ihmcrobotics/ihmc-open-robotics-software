package com.yobotics.simulationconstructionset.physics.collision.gdx;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import org.junit.Test;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3f;

import static org.junit.Assert.assertEquals;

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

		Transform3D src = new Transform3D();
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
		Transform3D dst = new Transform3D();

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
