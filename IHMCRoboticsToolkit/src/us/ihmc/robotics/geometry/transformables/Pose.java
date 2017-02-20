package us.ihmc.robotics.geometry.transformables;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class Pose extends AbstractPose implements GeometryObject<Pose>
{
   public Pose(Pose pose)
   {
      super(pose);
   }

   public Pose()
   {
      super();
   }

   public Pose(RigidBodyTransform transform)
   {
      super(transform);
   }

   public Pose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      super(position, orientation);
   }
   
   @Override
   public void set(Pose other)
   {
      setOrientation(other.getOrientation());
      setPosition(other.getPosition());
   }

   @Override
   public void setToZero()
   {
      setPoseToZero();
   }

   @Override
   public void setToNaN()
   {
      setPoseToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return poseContainsNaN();
   }

   @Override
   public boolean epsilonEquals(Pose other, double epsilon)
   {
      return epsilonEqualsPose(other, epsilon);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      applyTransformToPose(transform);
   }
   
   @Override
   public String toString()
   {
      return getPoseString();
   }
}
