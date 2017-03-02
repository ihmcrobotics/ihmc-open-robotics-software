package us.ihmc.simulationconstructionset.physics.collision.gdx;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * @author Peter Abeles
 */
public class GdxUtil
{
   public static void convert(RigidBodyTransform src, Matrix4 dst)
   {
      src.get(dst.getValues());
      dst.tra(); // transpose since it is column major
   }

   public static void convert(Matrix4 src, RigidBodyTransform dst)
   {
      // sets this transform to the transpose of src
      dst.setAsTranspose(src.getValues());
   }

   public static void convert(btVector3 src, Point3D dst)
   {
      dst.setX(src.x());
      dst.setY(src.y());
      dst.setZ(src.z());
   }

   public static void convert(Vector3 src, Point3D dst)
   {
      dst.setX(src.x);
      dst.setY(src.y);
      dst.setZ(src.z);
   }
}
