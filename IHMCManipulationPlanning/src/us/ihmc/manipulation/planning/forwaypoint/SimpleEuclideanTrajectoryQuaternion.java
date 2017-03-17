package us.ihmc.manipulation.planning.forwaypoint;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class SimpleEuclideanTrajectoryQuaternion
{
   private Quaternion orienatation = new Quaternion();
   private Vector3D angularVelocity = new Vector3D();
   
   public SimpleEuclideanTrajectoryQuaternion()
   {
      
   }
   
   public SimpleEuclideanTrajectoryQuaternion(Quaternion orientation)
   {
      this.orienatation.set(orientation);
   }
   
   public SimpleEuclideanTrajectoryQuaternion(Quaternion orientation, Vector3D angularVelocity)
   {
      this.orienatation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }
   
   public SimpleEuclideanTrajectoryQuaternion(SimpleEuclideanTrajectoryQuaternion trajectoryQuaternion)
   {
      this.orienatation.set(trajectoryQuaternion.getQuaternion());
      this.angularVelocity.set(trajectoryQuaternion.getAngularVelocity());
   }
   
   public void setQuaternion(Quaternion orientation)
   {
      this.orienatation.set(orientation);
   }
   
   public void setAngularVelocity(Vector3D angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }
   
   public Quaternion getQuaternion()
   {
      return this.orienatation;
   }
   
   public Vector3D getAngularVelocity()
   {
      return this.angularVelocity;
   }
}
