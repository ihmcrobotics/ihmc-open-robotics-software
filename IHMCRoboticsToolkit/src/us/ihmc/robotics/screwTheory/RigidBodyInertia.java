package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class RigidBodyInertia extends GeneralizedRigidBodyInertia
{
   private ReferenceFrame bodyFrame;    // (should maybe actually be a RigidBody, but this is how Duindam does it; can refactor later if needed)

   /**
    * Construct using the reference frame in which the RigidBodyInertia is expressed, the mass moment of inertia matrix in that frame and the mass
    * For the case that the origin of the frame in which the RigidBodyInertia is expressed coincides with the center of mass.
    *
    * @param frame the reference frame in which the RigidBodyInertia is expressed, and also the frame of which this
    * @param massMomentOfInertia the mass moment of inertia matrix in ReferenceFrame frame
    * @param mass the mass of the rigid body to which this RigidBodyInertia corresponds
    */
   public RigidBodyInertia(ReferenceFrame frame, Matrix3D massMomentOfInertia, double mass)
   {
      super(frame, massMomentOfInertia, mass);
      this.bodyFrame = frame;
   }

   public RigidBodyInertia(ReferenceFrame frame, double Ixx, double Iyy, double Izz, double mass)
   {
      super(frame, Ixx, Iyy, Izz, mass);
      this.bodyFrame = frame;
   }

   public RigidBodyInertia(ReferenceFrame frame, Matrix3D massMomentOfInertia, double mass, Vector3D crossPart)
   {
      super(frame, massMomentOfInertia, mass, crossPart);
      this.bodyFrame = frame;
   }

   /**
    * Copy constructor
    *
    * @param other the RigidBodyInertia to be copied
    */
   public RigidBodyInertia(RigidBodyInertia other)
   {
      super(other);
      this.bodyFrame = other.bodyFrame;
   }
   
   /**
    * @return the frame of which this inertia describes the inertia
    */
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   public void computeDynamicWrenchInBodyCoordinates(SpatialAccelerationVector acceleration, Twist twist, Wrench dynamicWrenchToPack)    // TODO: write test
   {
      checkExpressedInBodyFixedFrame();
      checkIsCrossPartZero();    // otherwise this operation would be a lot less efficient

      acceleration.getBodyFrame().checkReferenceFrameMatch(this.bodyFrame);
      acceleration.getBaseFrame().checkIsWorldFrame();
      acceleration.getExpressedInFrame().checkReferenceFrameMatch(this.expressedInframe);

      twist.getBodyFrame().checkReferenceFrameMatch(this.bodyFrame);
      twist.getBaseFrame().checkIsWorldFrame();
      twist.getExpressedInFrame().checkReferenceFrameMatch(this.expressedInframe);

      dynamicWrenchToPack.getBodyFrame().checkReferenceFrameMatch(this.bodyFrame);
      dynamicWrenchToPack.getExpressedInFrame().checkReferenceFrameMatch(this.expressedInframe);

      massMomentOfInertiaPart.transform(acceleration.getAngularPart(), tempVector);    // J * omegad
      dynamicWrenchToPack.setAngularPart(tempVector);    // [J * omegad; 0]

      massMomentOfInertiaPart.transform(twist.getAngularPart(), tempVector);    // J * omega
      tempVector.cross(twist.getAngularPart(), tempVector);    // omega x J * omega
      dynamicWrenchToPack.addAngularPart(tempVector);    // [J * omegad + omega x J * omega; 0]

      tempVector.set(acceleration.getLinearPart());    // vd
      tempVector.scale(mass);    // m * vd
      dynamicWrenchToPack.setLinearPart(tempVector);    // [J * omegad + omega x J * omega; m * vd]

      tempVector.set(twist.getLinearPart());    // v
      tempVector.scale(mass);    // m * v
      tempVector.cross(twist.getAngularPart(), tempVector);    // omega x m * v
      dynamicWrenchToPack.addLinearPart(tempVector);    // [J * omegad + omega x J * omega; m * vd + omega x m * v]
   }

   /**
    * Computes the kinetic co-energy of the rigid body to which this inertia belongs
    * twistTranspose * Inertia * twist
    *
    * See Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, page 40, eq. (2.56)
    * http://sites.google.com/site/vincentduindam/publications
    *
    * @param twist
    * @return
    */
   public double computeKineticCoEnergy(Twist twist)
   {
      this.expressedInframe.checkReferenceFrameMatch(twist.getExpressedInFrame());
      this.bodyFrame.checkReferenceFrameMatch(twist.getBodyFrame());
      twist.getBaseFrame().checkIsWorldFrame();

      double ret = 0.0;

      tempVector.set(twist.getAngularPart());    // omega
      massMomentOfInertiaPart.transform(tempVector);    // J * omega
      ret += twist.getAngularPart().dot(tempVector);    // omega . J * omega

      tempVector.set(twist.getAngularPart());    // omega
      tempVector.cross(crossPart, tempVector);    // c x omega
      ret += 2.0 * twist.getLinearPart().dot(tempVector);    // omega . J * omega + 2 * v . (c x omega)

      ret += mass * twist.getLinearPart().dot(twist.getLinearPart());    // omega . J * omega + 2 * v . (c x omega) + m * v . v

      return ret;
   }
   
   public void set(RigidBodyInertia other)
   {
      super.set(other);
      this.bodyFrame = other.bodyFrame;
   }

   private void checkExpressedInBodyFixedFrame()
   {
      this.bodyFrame.checkReferenceFrameMatch(this.expressedInframe);
   }
}
