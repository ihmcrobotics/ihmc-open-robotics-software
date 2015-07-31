package us.ihmc.simulationconstructionset.physics.collision.gdx;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;

/**
 * @author Peter Abeles
 */
public class GdxUtil
{
	public static void convert( RigidBodyTransform src , Matrix4 dst )  {
		src.get(dst.getValues());
		dst.tra(); // transpose since it is column major
	}

	public static void convert( Matrix4 src , RigidBodyTransform dst )  {
		// sets this transform to the transpose of src
	   dst.setAsTranspose(src.getValues());
	}

	public static void convert( btVector3 src , Point3d dst )  {
		dst.setX(src.x());
		dst.setY(src.y());
		dst.setZ(src.z());
	}
}
