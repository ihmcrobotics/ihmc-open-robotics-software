package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Momentum extends SpatialForce
{
   /**
    * Default constructor. Sets reference frames to null and angular and linear parts to zero.
    */
   public Momentum()
   {
      super();
   }

   /**
    * Construct using a double array ([angular; linear])
    */
   public Momentum(ReferenceFrame expressedInFrame, double[] momentum)
   {
      super(expressedInFrame, momentum);
   }

   /**
    * Construct using a Matrix ([angular; linear])
    */
   public Momentum(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      super(expressedInFrame, matrix);
   }

   /**
    * @param expressedInFrame the frame in which the momentum is expressed
    * @param angularPart angular momentum
    * @param linearPart linear momentum
    */
   public Momentum(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      super(expressedInFrame, angularPart, linearPart);
   }

   /**
    * Initializes the components of the momentum to zero
    * @param expressedInFrame the frame in which the momentum is expressed
    */
   public Momentum(ReferenceFrame expressedInFrame)
   {
      super(expressedInFrame);
   }

   /**
    * Copy constructor
    */
   public Momentum(Momentum other)
   {
      super(other);
   }

   /**
    * Sets this momentum to inertia * twist
    */
   public void compute(GeneralizedRigidBodyInertia inertia, Twist twist)
   {
      ReferenceFrame frame = inertia.getReferenceFrame();
      frame.checkReferenceFrameMatch(twist.getReferenceFrame());

      tempVector.set(twist.getAngularPart());
      inertia.massMomentOfInertiaPart.transform(tempVector);    // J * omegad
      getAngularPart().set(tempVector);

      if (!inertia.isCrossPartZero())
      {
         tempVector.cross(twist.getLinearPart(), inertia.crossPart);
         this.getAngularPart().add(tempVector);    // J * omegad - c x vd
      }

      tempVector.set(twist.getLinearPart());
      tempVector.scale(inertia.getMass());
      getLinearPart().set(tempVector);    // m * vd

      if (!inertia.isCrossPartZero())
      {
         tempVector.cross(inertia.crossPart, twist.getAngularPart());
         this.getLinearPart().add(tempVector);    // m * vd + c x omegad
      }

      this.expressedInFrame = frame;
   }

   public double computeKineticCoEnergy(Twist twist)
   {
      getReferenceFrame().checkReferenceFrameMatch(twist.getReferenceFrame());
      double ret = getLinearPart().dot(twist.getLinearPart()) + getAngularPart().dot(twist.getAngularPart());

      return ret;
   }
   
   @Override
   public String toString()
   {
      String ret = new String("Momentum expressed in frame " + expressedInFrame + "\n" + "Angular part: " + getAngularPart() + "\n"
                              + "Linear part: " + getLinearPart());

      return ret;
   }
}
