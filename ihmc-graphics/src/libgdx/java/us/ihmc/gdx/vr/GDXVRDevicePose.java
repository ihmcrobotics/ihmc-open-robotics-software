package us.ihmc.gdx.vr;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;

/**
 * Represents the pose of a {@link GDXVRDevice}, including its
 * transform, velocity and angular velocity. Also indicates
 * whether the pose is valid and whether the device is connected.
 */
public class GDXVRDevicePose
{
   private final Matrix4 transform = new Matrix4();
   private final Vector3 velocity = new Vector3();
   private final Vector3 angularVelocity = new Vector3();
   private boolean isValid;
   private boolean isConnected;
   /** the device index */
   private final int index;

   public GDXVRDevicePose(int index)
   {
      this.index = index;
   }

   public int getIndex()
   {
      return index;
   }

   /**
    * transform encoding the position and rotation of the device in tracker space
    **/
   public Matrix4 getTransform()
   {
      return transform;
   }

   /**
    * the velocity in m/s in tracker space space
    **/
   public Vector3 getVelocity()
   {
      return velocity;
   }

   /**
    * the angular velocity in radians/s in tracker space
    **/
   public Vector3 getAngularVelocity()
   {
      return angularVelocity;
   }

   /**
    * whether the pose is valid our invalid, e.g. outdated because of tracking failure
    **/
   public boolean isValid()
   {
      return isValid;
   }

   /**
    * whether the device is connected
    **/
   public boolean isConnected()
   {
      return isConnected;
   }

   public void setValid(boolean valid)
   {
      isValid = valid;
   }

   public void setConnected(boolean connected)
   {
      isConnected = connected;
   }
}
