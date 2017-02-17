package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CompositeRigidBodyInertia extends GeneralizedRigidBodyInertia
{   
   public CompositeRigidBodyInertia()
   {
      super();
   }

   public CompositeRigidBodyInertia(ReferenceFrame frame, Matrix3D massMomentOfInertia, double mass)
   {
      super(frame, massMomentOfInertia, mass);
   }

   public CompositeRigidBodyInertia(ReferenceFrame frame, double Ixx, double Iyy, double Izz, double mass)
   {
      super(frame, Ixx, Iyy, Izz, mass);
   }

   public CompositeRigidBodyInertia(ReferenceFrame frame, Matrix3D massMomentOfInertia, double mass, Vector3D crossPart)
   {
      super(frame, massMomentOfInertia, mass, crossPart);
   }

   public CompositeRigidBodyInertia(CompositeRigidBodyInertia other)
   {
      super(other);
   }

   public void add(CompositeRigidBodyInertia other)
   {
      expressedInframe.checkReferenceFrameMatch(other.expressedInframe);

      massMomentOfInertiaPart.add(other.massMomentOfInertiaPart);
      crossPart.add(other.crossPart);
      mass += other.mass;

      if (!this.isCrossPartZero() && !other.isCrossPartZero())
      {
         // since the crossParts might cancel after addition:
         determineIfCrossPartIsZero();
      }
      else
      {
         // if either inertia's crossPart is not zero, our crossPart will not be zero, otherwise it will be zero
         crossPartZero = this.isCrossPartZero() && other.isCrossPartZero();
      }
   }
}
