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
   private final VectorEnum<EffortVariableConstraintType> forceConstraintType = new VectorEnum<>();
   private final FrameVector3D forceWeight;

   private final FrameVector3D forceRate;
   private final VectorEnum<EffortVariableConstraintType> forceRateConstraintType = new VectorEnum<>();
   private final FrameVector3D forceRateWeight;

   private final FramePoint3D position;
   private final VectorEnum<DependentVariableConstraintType> positionConstraintType = new VectorEnum<>();
   private final FrameVector3D positionWeight;

   private final FrameVector3D linearVelocity;
   private final VectorEnum<DependentVariableConstraintType> linearVelocityConstraintType = new VectorEnum<>();
   private final FrameVector3D linearVelocityWeight;

   private final FrameVector3D torque;
   private final VectorEnum<DependentVariableConstraintType> torqueConstraintType = new VectorEnum<>();
   private final FrameVector3D torqueWeight;

   private final FrameVector3D angularVelocity;
   private final VectorEnum<DependentVariableConstraintType> angularVelocityConstraintType = new VectorEnum<>();
   private final FrameVector3D angularVelocityWeight;

   private final FrameQuaternion orientation;
   private final VectorEnum<DependentVariableConstraintType> orientationConstraintType = new VectorEnum<>();
   private final FrameVector3D orientationWeight;

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

      position = new FramePoint3D(referenceFrame);
      positionWeight = new FrameVector3D(referenceFrame);

      linearVelocity = new FrameVector3D(referenceFrame);
      linearVelocityWeight = new FrameVector3D(referenceFrame);

      force = new FrameVector3D(referenceFrame);
      forceWeight = new FrameVector3D(referenceFrame);

      forceRate = new FrameVector3D(referenceFrame);
      forceRateWeight = new FrameVector3D(referenceFrame);

      orientation = new FrameQuaternion(referenceFrame);
      orientationWeight = new FrameVector3D(referenceFrame);

      angularVelocity = new FrameVector3D(referenceFrame);
      angularVelocityWeight = new FrameVector3D(referenceFrame);

      torque = new FrameVector3D(referenceFrame);
      torqueWeight = new FrameVector3D(referenceFrame);

      reset();
   }

   public void reset()
   {
      time = Double.NaN;

      position.setToNaN();
      positionWeight.setToNaN();
      positionConstraintType.setXYZ(DependentVariableConstraintType.IGNORE);

      linearVelocity.setToNaN();
      linearVelocityWeight.setToNaN();
      linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.IGNORE);

      force.setToNaN();
      forceWeight.setToNaN();
      forceConstraintType.setToNull();

      forceRate.setToNaN();
      forceRateWeight.setToNaN();
      forceRateConstraintType.setToNull();

      orientation.setToNaN();
      orientationWeight.setToNaN();
      orientationConstraintType.setXYZ(DependentVariableConstraintType.IGNORE);

      angularVelocity.setToNaN();
      angularVelocityWeight.setToNaN();
      angularVelocityConstraintType.setXYZ(DependentVariableConstraintType.IGNORE);

      torque.setToNaN();
      torqueWeight.setToNaN();
      torqueConstraintType.setXYZ(DependentVariableConstraintType.IGNORE);
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public void setForceConstraint(FrameVector3D force)
   {
      setForceValue(force);
      this.forceConstraintType.setXYZ(EffortVariableConstraintType.CONSTRAINT);
      this.forceWeight.setToNaN();
   }

   public void setForceObjective(FrameVector3D force, FrameVector3D forceWeight)
   {
      setForceValue(force);
      this.forceConstraintType.setXYZ(EffortVariableConstraintType.OBJECTIVE);
      this.forceWeight.setIncludingFrame(forceWeight);
      this.forceWeight.changeFrame(referenceFrame);
   }

   public void setForce(FrameVector3D force, VectorEnum<EffortVariableConstraintType> constraintType, FrameVector3D forceWeight)
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

      forceConstraintType.setZ(EffortVariableConstraintType.OBJECTIVE);

      forceWeight.changeFrame(frame);
      forceWeight.setZ(weight);
      forceWeight.changeFrame(referenceFrame);
   }

   public void setForceRateObjective(FrameVector3D dForce, FrameVector3D dForceWeight)
   {
      this.forceRate.setIncludingFrame(dForce);
      this.changeReferenceFrame(referenceFrame);
      this.forceRateConstraintType.setXYZ(EffortVariableConstraintType.OBJECTIVE);
      this.forceRateWeight.setIncludingFrame(dForceWeight);
      this.forceRateWeight.changeFrame(referenceFrame);
   }

   public void setForceRateConstraint(FrameVector3D dForce)
   {
      this.forceRate.setIncludingFrame(dForce);
      this.forceRate.changeFrame(referenceFrame);
      this.forceRateConstraintType.setXYZ(EffortVariableConstraintType.CONSTRAINT);
      this.forceRateWeight.setToNaN();
   }

   public void setForceRate(FrameVector3D dForce, VectorEnum<EffortVariableConstraintType> constraintType, FrameVector3D dForceWeight)
   {
      this.forceRate.setIncludingFrame(dForce);
      this.forceRate.changeFrame(referenceFrame);
      this.forceRateConstraintType.set(constraintType);
      this.forceRateWeight.setIncludingFrame(dForceWeight);
      this.forceRateWeight.changeFrame(referenceFrame);
   }

   public void setTorqueObjective(FrameVector3D torque, FrameVector3D torqueWeight)
   {
      this.torque.setIncludingFrame(torque);
      this.torque.changeFrame(referenceFrame);
      this.torqueConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.torqueWeight.setIncludingFrame(torqueWeight);
      this.torqueWeight.changeFrame(referenceFrame);
   }

   public void setTorqueConstraint(FrameVector3D torque)
   {
      this.torque.setIncludingFrame(torque);
      this.torque.changeFrame(referenceFrame);
      this.torqueConstraintType.setXYZ(DependentVariableConstraintType.CONSTRAINT);
      this.torqueWeight.setToNaN();
   }

   public void setTorque(FrameVector3D torque, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D weights)
   {
      this.torque.setIncludingFrame(torque);
      this.torque.changeFrame(referenceFrame);
      this.torqueConstraintType.set(constraintType);
      this.torqueWeight.setIncludingFrame(weights);
      this.torqueWeight.changeFrame(referenceFrame);
   }

   public void setPositionObjective(FramePoint3D desiredPosition, FrameVector3D positionWeight)
   {
      this.position.setIncludingFrame(desiredPosition);
      this.position.changeFrame(referenceFrame);
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.positionWeight.setIncludingFrame(positionWeight);
      this.positionWeight.changeFrame(referenceFrame);
   }

   public void setPositionConstraint(FramePoint3D desiredPosition)
   {
      this.position.setIncludingFrame(desiredPosition);
      this.position.changeFrame(referenceFrame);
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.CONSTRAINT);
      this.positionWeight.setToNaN();
   }

   public void setPosition(FramePoint3D desiredPosition, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D positionWeight)
   {
      this.position.setIncludingFrame(desiredPosition);
      this.position.changeFrame(referenceFrame);
      this.positionConstraintType.set(constraintType);
      this.positionWeight.setIncludingFrame(positionWeight);
      this.positionWeight.changeFrame(referenceFrame);
   }

   public void setOrientationObjective(FrameQuaternion desiredOrienation, FrameVector3D orientationWeight)
   {
      this.orientation.setIncludingFrame(desiredOrienation);
      this.orientation.changeFrame(referenceFrame);
      this.orientationConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.orientationWeight.setIncludingFrame(orientationWeight);
      this.orientationWeight.changeFrame(referenceFrame);
   }

   public void setOrientationConstraint(FrameQuaternion desiredOrienation)
   {
      this.orientation.setIncludingFrame(desiredOrienation);
      this.orientation.changeFrame(referenceFrame);
      this.orientationConstraintType.setXYZ(DependentVariableConstraintType.CONSTRAINT);
      this.orientationWeight.setToNaN();
   }

   public void setOrientation(FrameQuaternion desiredOrienation, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D orientationWeight)
   {
      this.orientation.setIncludingFrame(desiredOrienation);
      this.orientation.changeFrame(referenceFrame);
      this.orientationConstraintType.set(constraintType);
      this.orientationWeight.setIncludingFrame(orientationWeight);
      this.orientationWeight.changeFrame(referenceFrame);
   }

   public void setLinearVelocityObjective(FrameVector3D desiredLinearVelocity, FrameVector3D linearVelocityWeight)
   {
      this.linearVelocity.setIncludingFrame(desiredLinearVelocity);
      this.linearVelocity.changeFrame(referenceFrame);
      this.linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.linearVelocityWeight.setIncludingFrame(linearVelocityWeight);
      this.linearVelocityWeight.changeFrame(referenceFrame);
   }

   public void setLinearVelocityConstraint(FrameVector3D desiredLinearVelocity)
   {
      this.linearVelocity.setIncludingFrame(desiredLinearVelocity);
      this.linearVelocity.changeFrame(referenceFrame);
      this.linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.CONSTRAINT);
      this.linearVelocityWeight.setToNaN();
   }

   public void setLinearVelocityObjective(FrameVector3D desiredLinearVelocity, VectorEnum<DependentVariableConstraintType> constraintType,
                                          FrameVector3D linearVelocityWeight)
   {
      this.linearVelocity.setIncludingFrame(desiredLinearVelocity);
      this.linearVelocity.changeFrame(referenceFrame);
      this.linearVelocityConstraintType.set(constraintType);
      this.linearVelocityWeight.setIncludingFrame(linearVelocityWeight);
      this.linearVelocityWeight.changeFrame(referenceFrame);
   }

   public void setAngularVelocityObjective(FrameVector3D desiredAngularVelocity, FrameVector3D angularVelocityWeight)
   {
      this.angularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.angularVelocity.changeFrame(referenceFrame);
      this.angularVelocityConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.angularVelocityWeight.setIncludingFrame(angularVelocityWeight);
      this.angularVelocityWeight.changeFrame(referenceFrame);
   }

   public void setAngularVelocityConstraint(FrameVector3D desiredAngularVelocity)
   {
      this.angularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.angularVelocity.changeFrame(referenceFrame);
      this.angularVelocityConstraintType.setXYZ(DependentVariableConstraintType.CONSTRAINT);
      this.angularVelocityWeight.setToNaN();
   }

   public void setAngularVelocity(FrameVector3D desiredAngularVelocity, VectorEnum<DependentVariableConstraintType> constraintType,
                                  FrameVector3D angularVelocityWeight)
   {
      this.angularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.angularVelocity.changeFrame(referenceFrame);
      this.angularVelocityConstraintType.set(constraintType);
      this.angularVelocityWeight.setIncludingFrame(angularVelocityWeight);
      this.angularVelocityWeight.changeFrame(referenceFrame);
   }

   public double getTime()
   {
      return time;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void changeReferenceFrame(ReferenceFrame desiredFrame)
   {
      position.changeFrame(desiredFrame);
      positionWeight.changeFrame(desiredFrame);
      linearVelocity.changeFrame(desiredFrame);
      linearVelocityWeight.changeFrame(desiredFrame);

      force.changeFrame(desiredFrame);
      forceWeight.changeFrame(desiredFrame);

      forceRate.changeFrame(desiredFrame);
      forceRateWeight.changeFrame(desiredFrame);

      orientation.changeFrame(desiredFrame);
      orientationWeight.changeFrame(desiredFrame);

      angularVelocity.changeFrame(desiredFrame);
      angularVelocityWeight.changeFrame(desiredFrame);

      torque.changeFrame(desiredFrame);
      torqueWeight.changeFrame(desiredFrame);

      referenceFrame = desiredFrame;
   }

   public void set(CentroidalMotionNode other)
   {
      this.referenceFrame = other.referenceFrame;
      this.time = other.time;

      this.position.setIncludingFrame(other.position);
      this.positionWeight.setIncludingFrame(other.positionWeight);
      this.positionConstraintType.set(other.positionConstraintType);

      this.linearVelocity.setIncludingFrame(other.linearVelocity);
      this.linearVelocityWeight.setIncludingFrame(other.linearVelocityWeight);
      this.linearVelocityConstraintType.set(other.linearVelocityConstraintType);

      this.force.setIncludingFrame(other.force);
      this.forceWeight.setIncludingFrame(other.forceWeight);
      this.forceConstraintType.set(other.forceConstraintType);

      this.forceRate.setIncludingFrame(other.forceRate);
      this.forceRateWeight.setIncludingFrame(other.forceRateWeight);
      this.forceRateConstraintType.set(other.forceRateConstraintType);

      this.orientation.setIncludingFrame(other.orientation);
      this.orientationWeight.setIncludingFrame(other.orientationWeight);
      this.orientationConstraintType.set(other.orientationConstraintType);

      this.angularVelocity.setIncludingFrame(other.angularVelocity);
      this.angularVelocityWeight.setIncludingFrame(other.angularVelocityWeight);
      this.angularVelocityConstraintType.set(angularVelocityConstraintType);

      this.torque.setIncludingFrame(other.torque);
      this.torqueWeight.setIncludingFrame(other.torqueWeight);
      this.torqueConstraintType.set(other.torqueConstraintType);
   }

   public EffortVariableConstraintType getXForceConstraintType()
   {
      return forceConstraintType.getX();
   }

   public EffortVariableConstraintType getYForceConstraintType()
   {
      return forceConstraintType.getY();
   }

   public EffortVariableConstraintType getZForceConstraintType()
   {
      return forceConstraintType.getZ();
   }

   public EffortVariableConstraintType getForceConstraintType(Axis axis)
   {
      return forceConstraintType.getElement(axis);
   }

   public EffortVariableConstraintType getForceRateConstraintType(Axis axis)
   {
      return forceRateConstraintType.getElement(axis);
   }

   public double getPosition(Axis axis)
   {
      return position.getElement(axis.ordinal());
   }

   public double getPositionWeight(Axis axis)
   {
      return positionWeight.getElement(axis.ordinal());
   }

   public DependentVariableConstraintType getPositionConstraintType(Axis axis)
   {
      return positionConstraintType.getElement(axis);
   }

   public double getLinearVelocity(Axis axis)
   {
      return linearVelocity.getElement(axis.ordinal());
   }

   public double getLinearVelocityWeight(Axis axis)
   {
      return linearVelocityWeight.getElement(axis.ordinal());
   }

   public DependentVariableConstraintType getLinearVelocityConstraintType(Axis axis)
   {
      return linearVelocityConstraintType.getElement(axis);
   }

   public double getForce(Axis axis)
   {
      return force.getElement(axis.ordinal());
   }

   public double getForceRate(Axis axis)
   {
      return forceRate.getElement(axis.ordinal());
   }

   public double getForceWeight(Axis axis)
   {
      return forceWeight.getElement(axis.ordinal());
   }

   public double getForceRateWeight(Axis axis)
   {
      return forceRateWeight.getElement(axis.ordinal());
   }
}