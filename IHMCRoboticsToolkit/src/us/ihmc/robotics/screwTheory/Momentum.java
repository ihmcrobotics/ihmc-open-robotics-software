package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class Momentum extends SpatialForceVector
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
    * @param linearPart linear momentum
    * @param angularPart angular momentum
    */
   public Momentum(ReferenceFrame expressedInFrame, Vector3D linearPart, Vector3D angularPart)
   {
      super(expressedInFrame, linearPart, angularPart);
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
      ReferenceFrame frame = inertia.getExpressedInFrame();
      frame.checkReferenceFrameMatch(twist.getExpressedInFrame());

      tempVector.set(twist.getAngularPart());
      inertia.massMomentOfInertiaPart.transform(tempVector);    // J * omegad
      setAngularPart(tempVector);

      if (!inertia.isCrossPartZero())
      {
         tempVector.cross(twist.getLinearPart(), inertia.crossPart);
         addAngularPart(tempVector);    // J * omegad - c x vd
      }

      tempVector.set(twist.getLinearPart());
      tempVector.scale(inertia.getMass());
      setLinearPart(tempVector);    // m * vd

      if (!inertia.isCrossPartZero())
      {
         tempVector.cross(inertia.crossPart, twist.getAngularPart());
         addLinearPart(tempVector);    // m * vd + c x omegad
      }

      this.expressedInFrame = frame;
   }

   public double computeKineticCoEnergy(Twist twist)
   {
      getExpressedInFrame().checkReferenceFrameMatch(twist.getExpressedInFrame());
      double ret = linearPart.dot(twist.linearPart) + angularPart.dot(twist.angularPart);

      return ret;
   }
   
   @Override
   public String toString()
   {
      String ret = new String("Momentum expressed in frame " + expressedInFrame + "\n" + "Angular part: " + angularPart + "\n"
                              + "Linear part: " + linearPart);

      return ret;
   }
}
