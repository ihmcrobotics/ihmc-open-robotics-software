package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class Wrench extends SpatialForceVector
{
   ReferenceFrame bodyFrame;

   /**
    * Initializes the components of the wrench to zero
    * @param bodyFrame the frame/body on which the wrench is exerted
    * @param expressedInFrame the frame in which the wrench is expressed
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      super(expressedInFrame);
      this.bodyFrame = bodyFrame;
   }

   /**
    * @param bodyFrame the frame/body on which the wrench is exerted
    * @param expressedInFrame the frame in which the wrench is expressed
    * @param force force part of the wrench
    * @param torque torque part of the wrench
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3D force, Vector3D torque)
   {
      super(expressedInFrame, force, torque);
      this.bodyFrame = bodyFrame;
   }

   /**
    * Construct using a Matrix ([torque; force])
    * @param bodyFrame the frame/body on which the wrench is exerted
    * @param expressedInFrame the frame in which the wrench is expressed
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F wrench)
   {
      super(expressedInFrame, wrench);
      this.bodyFrame = bodyFrame;
   }

   /**
    * Construct using a double array ([torque; force])
    * @param bodyFrame the frame/body on which the wrench is exerted
    * @param expressedInFrame the frame in which the wrench is expressed
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double[] wrench)
   {
      super(expressedInFrame, wrench);
      this.bodyFrame = bodyFrame;
   }

   /**
    * Copy constructor
    */
   public Wrench(Wrench other)
   {
      super(other);
      this.bodyFrame = other.bodyFrame;
   }

   /**
    * Default constructor. Sets reference frames to null and angular and linear parts to zero.
    */
   public Wrench()
   {
      this(null, null);
   }

   /**
    * @return the frame/body *on which* this wrench is exerted
    */
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }
   
   public void checkAndSet(Wrench other)
   {
      this.bodyFrame.checkReferenceFrameMatch(other.bodyFrame);
      super.checkAndSet(other);
   }

   public void set(Wrench other)
   {
      this.bodyFrame = other.bodyFrame;
      super.set(other.expressedInFrame, other.linearPart, other.angularPart);
   }

   /**
    * Sets angular and linear parts using a Matrix ([torque; force]).
    * @param bodyFrame the frame/body on which the wrench is exerted
    * @param expressedInFrame the frame in which the wrench is expressed
    */
   public void set(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F wrench)
   {
      set(bodyFrame, expressedInFrame, wrench, 0);
   }

   public void set(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F wrench, int rowStart)
   {
      set(expressedInFrame, wrench, rowStart);
      this.bodyFrame = bodyFrame;
   }

   /**
    * Sets angular and linear parts to zero.
    * @param bodyFrame the frame/body on which the wrench is exerted
    * @param expressedInFrame the frame in which the wrench is expressed
    */
   public void setToZero(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      this.bodyFrame = bodyFrame;
      this.expressedInFrame = expressedInFrame;
      angularPart.set(0.0, 0.0, 0.0);
      linearPart.set(0.0, 0.0, 0.0);
   }

   /**
    * Adds another wrench to this one, after performing some reference frame checks.
    */
   public void add(Wrench other)
   {
      this.bodyFrame.checkReferenceFrameMatch(other.bodyFrame);
      super.add(other);
   }

   /**
    * Subtracts another wrench from this one, after performing some reference frame checks.
    */
   public void sub(Wrench other)
   {
      this.bodyFrame.checkReferenceFrameMatch(other.bodyFrame);
      super.sub(other);
   }

   public void changeBodyFrameAttachedToSameBody(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /**
    * Takes the dot product of this wrench and a twist, resulting in the (reference frame independent) instantaneous power.
    * @param twist a twist that
    *       1) has an 'ofWhat' reference frame that is the same as this wrench's 'bodyFrame' reference frame.
    *       2) is expressed in the same reference frame as this wrench
    * @return the instantaneous power associated with this wrench and the twist
    */
   public double dot(Twist twist)
   {
      return twist.dot(this);
   }

   @Override
   public String toString()
   {
      String ret = new String("Wrench exerted on " + bodyFrame + ", expressed in frame " + expressedInFrame + "\n" + "Torque part: " + angularPart + "\n"
                              + "Force part: " + linearPart);

      return ret;
   }
}
