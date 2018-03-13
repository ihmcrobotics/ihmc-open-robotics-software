package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * This class is used to store all the relevant data for a particular node of a trajectory.
 * Each node represents the beginning / end point of a trajectory and has the following attributes:
 * <li> {@code position}
 * <li> {@code linearVelocity}
 * <li> {@code force}
 * <li> {@code rateOfChangeOfForce}
 * <li> {@code orientation}
 * <li> {@code angularVelocity}
 * <li> {@code torque}
 * <li> {@code rateOfChangeOfTorque}
 * @author Apoorv S
 *
 */
public class CentroidalMotionNode implements ReferenceFrameHolder
{
   private double time;
   private ReferenceFrame referenceFrame;

   private FrameVector3D force;
   private FrameVector3D rateOfChangeOfForce;
   private FramePoint3D position;
   private FrameVector3D linearVelocity;

   private FrameVector3D torque;
   private FrameVector3D rateOfChangeOfTorque;
   private FrameQuaternion orientation;
   private FrameVector3D angularVelocity;

   private FrameVector3D positionWeight;
   private FrameVector3D linearVelocityWeight;
   private FrameVector3D forceWeight;
   private FrameVector3D orientationWeight;
   private FrameVector3D angularVelocityWeight;
   private FrameVector3D torqueWeight;
   private VectorEnum<EffortConstraintType> forceConstraintType = new VectorEnum<>();
   private VectorEnum<EffortConstraintType> torqueConstraintType = new VectorEnum<>();

   /**
    * Default constructor. Initialized all variables to NaN so that they form part of the optimization
    */
   public CentroidalMotionNode()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public CentroidalMotionNode(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      force = new FrameVector3D(referenceFrame);
      rateOfChangeOfForce = new FrameVector3D(referenceFrame);
      position = new FramePoint3D(referenceFrame);
      linearVelocity = new FrameVector3D(referenceFrame);

      torque = new FrameVector3D(referenceFrame);
      rateOfChangeOfTorque = new FrameVector3D(referenceFrame);
      orientation = new FrameQuaternion(referenceFrame);
      angularVelocity = new FrameVector3D(referenceFrame);

      positionWeight = new FrameVector3D(referenceFrame);
      linearVelocityWeight = new FrameVector3D(referenceFrame);
      forceWeight = new FrameVector3D(referenceFrame);

      orientationWeight = new FrameVector3D(referenceFrame);
      angularVelocityWeight = new FrameVector3D(referenceFrame);
      torque = new FrameVector3D(referenceFrame);

      reset();
   }

   public void reset()
   {
      time = Double.NaN;

      position.setToNaN();
      linearVelocity.setToNaN();
      force.setToNaN();
      rateOfChangeOfForce.setToNaN();

      orientation.setToNaN();
      angularVelocity.setToNaN();
      torque.setToNaN();
      rateOfChangeOfTorque.setToNaN();
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public void setForce(FrameVector3D force)
   {
      this.force.setIncludingFrame(force);
      this.force.changeFrame(referenceFrame);
   }

   public void setRateOfChangeOfForce(FrameVector3D dForce)
   {
      this.rateOfChangeOfForce.setIncludingFrame(dForce);
      this.rateOfChangeOfForce.changeFrame(referenceFrame);
   }

   public void setTorque(FrameVector3D torque)
   {
      this.torque.setIncludingFrame(torque);
      this.torque.changeFrame(referenceFrame);
   }

   public void setRateOfChangeOfTorque(FrameVector3D dTorque)
   {
      this.rateOfChangeOfTorque.setIncludingFrame(dTorque);
      this.rateOfChangeOfTorque.changeFrame(referenceFrame);
   }

   public void setPositionObjective(FramePoint3D desiredPosition)
   {
      this.position.setIncludingFrame(desiredPosition);
      this.position.changeFrame(referenceFrame);
   }

   public void setOrientationObjective(FrameQuaternion desiredOrienation)
   {
      this.orientation.setIncludingFrame(desiredOrienation);
      this.orientation.changeFrame(referenceFrame);
   }

   public void setLinearVeclocityObjective(FrameVector3D desiredLinearVelocity)
   {
      this.linearVelocity.setIncludingFrame(desiredLinearVelocity);
      this.linearVelocity.changeFrame(referenceFrame);
   }

   public void setAngularVelocity(FrameVector3D desiredAngularVelocity)
   {
      this.angularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.angularVelocity.changeFrame(referenceFrame);
   }

   public double getTime()
   {
      return time;
   }

   public void getDesiredPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void changeReferenceFrame(ReferenceFrame desiredFrame)
   {
      position.changeFrame(desiredFrame);
      linearVelocity.changeFrame(desiredFrame);
      force.changeFrame(desiredFrame);
      rateOfChangeOfForce.changeFrame(desiredFrame);

      orientation.changeFrame(desiredFrame);
      angularVelocity.changeFrame(desiredFrame);
      torque.changeFrame(desiredFrame);
      rateOfChangeOfTorque.changeFrame(desiredFrame);

      positionWeight.changeFrame(desiredFrame);
      orientationWeight.changeFrame(desiredFrame);
      forceWeight.changeFrame(desiredFrame);

      linearVelocityWeight.changeFrame(desiredFrame);
      angularVelocityWeight.changeFrame(desiredFrame);
      torqueWeight.changeFrame(desiredFrame);

      referenceFrame = desiredFrame;
   }
}