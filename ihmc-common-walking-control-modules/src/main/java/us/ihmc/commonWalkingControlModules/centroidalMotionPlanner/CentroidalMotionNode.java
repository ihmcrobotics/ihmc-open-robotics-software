package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.Axis;
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

   private final FrameVector3D force;
   private final FrameVector3D rateOfChangeOfForce;
   private final FramePoint3D position;
   private final FrameVector3D linearVelocity;

   private final FrameVector3D torque;
   private final FrameVector3D rateOfChangeOfTorque;
   private final FrameQuaternion orientation;
   private final FrameVector3D angularVelocity;

   private final FrameVector3D positionWeight;
   private final FrameVector3D linearVelocityWeight;
   private final FrameVector3D forceWeight;
   private final FrameVector3D rateChangeForceWeight;
   private final FrameVector3D orientationWeight;
   private final FrameVector3D angularVelocityWeight;
   private final FrameVector3D torqueWeight;
   private final VectorEnum<EffortConstraintType> forceConstraintType = new VectorEnum<>();
   private final VectorEnum<EffortConstraintType> rateChangeForceConstraintType = new VectorEnum<>();

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
      rateChangeForceWeight = new FrameVector3D(referenceFrame);

      orientationWeight = new FrameVector3D(referenceFrame);
      angularVelocityWeight = new FrameVector3D(referenceFrame);
      torqueWeight = new FrameVector3D(referenceFrame);

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

      forceConstraintType.setToNull();
      rateChangeForceConstraintType.setToNull();

      positionWeight.setToNaN();
      linearVelocityWeight.setToNaN();
      forceWeight.setToNaN();
      rateChangeForceWeight.setToNaN();

      orientationWeight.setToNaN();
      angularVelocityWeight.setToNaN();
      torqueWeight.setToNaN();
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public void setForceAsHardConstraint(FrameVector3D force)
   {
      setForceValue(force);
      this.forceConstraintType.setXYZ(EffortConstraintType.EQUALITY);
      this.forceWeight.setToNaN();
   }

   public void setForceAsObjective(FrameVector3D force, FrameVector3D forceWeight)
   {
      setForceValue(force);
      this.forceConstraintType.setXYZ(EffortConstraintType.OBJECTIVE);
      this.forceWeight.setIncludingFrame(forceWeight);
      this.forceWeight.changeFrame(referenceFrame);
   }

   public void setForce(FrameVector3D force, VectorEnum<EffortConstraintType> constraintType, FrameVector3D forceWeight)
   {
      setForceValue(force);
      this.forceConstraintType.set(constraintType);
      this.forceWeight.setIncludingFrame(forceWeight);
      this.forceWeight.changeFrame(referenceFrame);
   }

   private void setForceValue(FrameVector3D force)
   {
      this.force.setIncludingFrame(force);
      this.force.changeFrame(referenceFrame);
   }

   public void setForceZ(ReferenceFrame frame, double fZ, double weight)
   {
      force.changeFrame(frame);
      force.setZ(fZ);
      force.changeFrame(referenceFrame);

      forceConstraintType.setZ(EffortConstraintType.OBJECTIVE);

      forceWeight.changeFrame(frame);
      forceWeight.setZ(weight);
      forceWeight.changeFrame(referenceFrame);
   }

   public void setRateOfChangeOfForce(FrameVector3D dForce, VectorEnum<EffortConstraintType> constraintType, FrameVector3D dForceWeight)
   {
      this.rateOfChangeOfForce.setIncludingFrame(dForce);
      this.rateOfChangeOfForce.changeFrame(referenceFrame);
      this.rateChangeForceConstraintType.set(constraintType);
      this.rateChangeForceWeight.setIncludingFrame(dForceWeight);
      this.rateChangeForceWeight.changeFrame(referenceFrame);
   }

   public void setRateOfChangeOfForceAsHardConstraint(FrameVector3D dForce)
   {
      this.rateOfChangeOfForce.setIncludingFrame(dForce);
      this.rateOfChangeOfForce.changeFrame(referenceFrame);
      this.rateChangeForceConstraintType.setXYZ(EffortConstraintType.EQUALITY);
      this.rateChangeForceWeight.setToNaN();
   }

   public void setRateOfChangeOfForceAsObjective(FrameVector3D dForce, FrameVector3D dForceWeight)
   {
      this.rateOfChangeOfForce.setIncludingFrame(dForce);
      this.changeReferenceFrame(referenceFrame);
      this.rateChangeForceConstraintType.setXYZ(EffortConstraintType.OBJECTIVE);
      this.rateChangeForceWeight.setIncludingFrame(dForceWeight);
      this.rateChangeForceWeight.changeFrame(referenceFrame);
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

   public void setPositionObjective(FramePoint3D desiredPosition, FrameVector3D positionWeight)
   {
      this.position.setIncludingFrame(desiredPosition);
      this.position.changeFrame(referenceFrame);

      this.positionWeight.setIncludingFrame(positionWeight);
      this.positionWeight.changeFrame(referenceFrame);
   }

   public void setOrientationObjective(FrameQuaternion desiredOrienation, FrameVector3D orientationWeight)
   {
      this.orientation.setIncludingFrame(desiredOrienation);
      this.orientation.changeFrame(referenceFrame);

      this.orientationWeight.setIncludingFrame(orientationWeight);
      this.orientationWeight.changeFrame(referenceFrame);
   }

   public void setLinearVeclocityObjective(FrameVector3D desiredLinearVelocity, FrameVector3D linearVelocityWeight)
   {
      this.linearVelocity.setIncludingFrame(desiredLinearVelocity);
      this.linearVelocity.changeFrame(referenceFrame);

      this.linearVelocityWeight.setIncludingFrame(linearVelocityWeight);
      this.linearVelocityWeight.changeFrame(referenceFrame);
   }

   public void setAngularVelocity(FrameVector3D desiredAngularVelocity, FrameVector3D angularVelocityWeight)
   {
      this.angularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.angularVelocity.changeFrame(referenceFrame);

      this.angularVelocityWeight.setIncludingFrame(angularVelocityWeight);
      this.angularVelocityWeight.changeFrame(referenceFrame);
   }

   public double getTime()
   {
      return time;
   }

   public void getPosition(FramePoint3D positionToPack)
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

   public void set(CentroidalMotionNode other)
   {
      this.referenceFrame = other.referenceFrame;
      this.time = other.time;

      this.position.setIncludingFrame(other.position);
      this.linearVelocity.setIncludingFrame(other.linearVelocity);
      this.force.setIncludingFrame(other.force);
      this.rateOfChangeOfForce.setIncludingFrame(other.rateOfChangeOfForce);

      this.orientation.setIncludingFrame(other.orientation);
      this.angularVelocity.setIncludingFrame(other.angularVelocity);
      this.torque.setIncludingFrame(other.torque);
      this.rateOfChangeOfTorque.setIncludingFrame(other.rateOfChangeOfTorque);

      this.forceConstraintType.set(other.forceConstraintType);
      this.rateChangeForceConstraintType.set(other.rateChangeForceConstraintType);

      this.positionWeight.setIncludingFrame(other.positionWeight);
      this.linearVelocityWeight.setIncludingFrame(other.linearVelocityWeight);
      this.forceWeight.setIncludingFrame(other.forceWeight);

      this.orientationWeight.setIncludingFrame(other.orientationWeight);
      this.angularVelocityWeight.setIncludingFrame(other.angularVelocityWeight);
      this.torqueWeight.setIncludingFrame(other.torqueWeight);
   }

   public EffortConstraintType getXForceConstraintType()
   {
      return forceConstraintType.getX();
   }

   public EffortConstraintType getYForceConstraintType()
   {
      return forceConstraintType.getY();
   }

   public EffortConstraintType getZForceConstraintType()
   {
      return forceConstraintType.getZ();
   }

   public EffortConstraintType getForceConstraintType(Axis axis)
   {
      return forceConstraintType.getElement(axis);
   }

   public EffortConstraintType getRateChangeForceConstraintType(Axis axis)
   {
      return rateChangeForceConstraintType.getElement(axis);
   }

   public double getPosition(Axis axis)
   {
      return position.getElement(axis.ordinal());
   }

   public double getLinearVelocity(Axis axis)
   {
      return linearVelocity.getElement(axis.ordinal());
   }

   public double getForceValue(Axis axis)
   {
      return force.getElement(axis.ordinal());
   }

   public double getRateChangeOfForceValue(Axis axis)
   {
      return rateOfChangeOfForce.getElement(axis.ordinal());
   }
}