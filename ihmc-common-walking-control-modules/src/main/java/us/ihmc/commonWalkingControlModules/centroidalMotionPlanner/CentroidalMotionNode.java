package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

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
   private final FramePoint3D positionMax;
   private final FramePoint3D positionMin;

   private final FrameVector3D linearVelocity;
   private final VectorEnum<DependentVariableConstraintType> linearVelocityConstraintType = new VectorEnum<>();
   private final FrameVector3D linearVelocityWeight;
   private final FrameVector3D linearVelocityMax;
   private final FrameVector3D linearVelocityMin;

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
      positionMin = new FramePoint3D(referenceFrame);
      positionMax = new FramePoint3D(referenceFrame);

      linearVelocity = new FrameVector3D(referenceFrame);
      linearVelocityWeight = new FrameVector3D(referenceFrame);
      linearVelocityMin = new FrameVector3D(referenceFrame);
      linearVelocityMax = new FrameVector3D(referenceFrame);

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
      positionMin.setToNaN();
      positionMax.setToNaN();

      linearVelocity.setToNaN();
      linearVelocityWeight.setToNaN();
      linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.IGNORE);
      linearVelocityMin.setToNaN();
      linearVelocityMax.setToNaN();

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
      setForceInternal(force);
      this.forceConstraintType.setXYZ(EffortVariableConstraintType.EQUALITY);
      this.forceWeight.setToNaN();
   }

   public void setForceObjective(FrameVector3D force, FrameVector3D forceWeight)
   {
      setForceInternal(force);
      this.forceConstraintType.setXYZ(EffortVariableConstraintType.OBJECTIVE);
      setForceWeightInternal(forceWeight);
   }

   private void setForceWeightInternal(FrameVector3D forceWeight)
   {
      this.forceWeight.setIncludingFrame(forceWeight);
      this.forceWeight.changeFrame(referenceFrame);
   }

   public void setForce(FrameVector3D force, VectorEnum<EffortVariableConstraintType> constraintType, FrameVector3D forceWeight)
   {
      setForceInternal(force);
      this.forceConstraintType.set(constraintType);
      setForceWeightInternal(forceWeight);
   }

   private void setForceInternal(FrameVector3D force)
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
      setForceRateInternal(dForce);
      this.forceRateConstraintType.setXYZ(EffortVariableConstraintType.OBJECTIVE);
      setForceRateWeightInternal(dForceWeight);
   }

   private void setForceRateWeightInternal(FrameVector3D dForceWeight)
   {
      this.forceRateWeight.setIncludingFrame(dForceWeight);
      this.forceRateWeight.changeFrame(referenceFrame);
   }

   private void setForceRateInternal(FrameVector3D dForce)
   {
      this.forceRate.setIncludingFrame(dForce);
      this.changeReferenceFrame(referenceFrame);
   }

   public void setForceRateConstraint(FrameVector3D dForce)
   {
      setForceRateInternal(dForce);
      this.forceRateConstraintType.setXYZ(EffortVariableConstraintType.EQUALITY);
      this.forceRateWeight.setToNaN();
   }

   public void setForceRate(FrameVector3D dForceRate, VectorEnum<EffortVariableConstraintType> constraintType, FrameVector3D dForceRateWeight)
   {
      setForceRateInternal(dForceRate);
      this.forceRateConstraintType.set(constraintType);
      setForceRateWeightInternal(dForceRateWeight);
   }

   public void setTorqueObjective(FrameVector3D torque, FrameVector3D torqueWeight)
   {
      setTorqueInternal(torque);
      this.torqueConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setTorqueWeightInternal(torqueWeight);
   }

   private void setTorqueWeightInternal(FrameVector3D torqueWeight)
   {
      this.torqueWeight.setIncludingFrame(torqueWeight);
      this.torqueWeight.changeFrame(referenceFrame);
   }

   private void setTorqueInternal(FrameVector3D torque)
   {
      this.torque.setIncludingFrame(torque);
      this.torque.changeFrame(referenceFrame);
   }

   public void setTorqueConstraint(FrameVector3D torque)
   {
      setTorqueInternal(torque);
      this.torqueConstraintType.setXYZ(DependentVariableConstraintType.EQUALITY);
      this.torqueWeight.setToNaN();
   }

   public void setTorque(FrameVector3D torque, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D weights)
   {
      setTorqueInternal(torque);
      this.torqueConstraintType.set(constraintType);
      setTorqueWeightInternal(weights);
   }

   public void setPositionObjective(FramePoint3D desiredPosition, FrameVector3D positionWeight)
   {
      setPositionInternal(desiredPosition);
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setPositionWeightInternal(positionWeight);
      this.positionMax.setToNaN();
      this.positionMin.setToNaN();
   }

   public void setPositionObjective(FramePoint3D desiredPosition, FrameVector3D positionWeight, FramePoint3D maxPosition, FramePoint3D minPosition)
   {
      setPositionInternal(desiredPosition);
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setPositionWeightInternal(positionWeight);
      setPositionMaxInternal(maxPosition);
      setPositionMinInternal(minPosition);
   }

   public void setPositionInequalitiesForObjective(FramePoint3D maxPosition, FramePoint3D minPosition)
   {
      setPositionMaxInternal(maxPosition);
      setPositionMinInternal(minPosition);
   }

   public void setPositionInequalities(FramePoint3D maxPosition, FramePoint3D minPosition)
   {
      this.position.setToNaN();
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.positionWeight.setToNaN();
      setPositionMaxInternal(maxPosition);
      setPositionMinInternal(minPosition);
   }

   public void setPositionMaxInequality(FramePoint3D maxPosition)
   {
      this.position.setToNaN();
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.positionWeight.setToNaN();
      setPositionMaxInternal(maxPosition);
      this.positionMin.setToNaN();
   }

   public void setPositionMinInequality(FramePoint3D minPosition)
   {
      this.position.setToNaN();
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.positionWeight.setToNaN();
      this.positionMax.setToNaN();
      setPositionMinInternal(minPosition);
   }

   public void setPositionConstraint(FramePoint3D desiredPosition)
   {
      setPositionInternal(desiredPosition);
      this.positionConstraintType.setXYZ(DependentVariableConstraintType.EQUALITY);
      this.positionWeight.setToNaN();
      this.positionMax.setToNaN();
      this.positionMin.setToNaN();
   }

   public void setPosition(FramePoint3D desiredPosition, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D positionWeight,
                           FramePoint3D maxPosition, FramePoint3D minPosition)
   {
      setPositionInternal(desiredPosition);
      this.positionConstraintType.set(constraintType);
      setPositionWeightInternal(positionWeight);
      setPositionMaxInternal(maxPosition);
      setPositionMinInternal(minPosition);
   }

   public void setPosition(FramePoint3D desiredPosition, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D positionWeight)
   {
      setPositionInternal(desiredPosition);
      this.positionConstraintType.set(constraintType);
      setPositionWeightInternal(positionWeight);
      positionMax.setToNaN();
      positionMin.setToNaN();
   }

   private void setPositionWeightInternal(FrameVector3D positionWeight)
   {
      this.positionWeight.setIncludingFrame(positionWeight);
      this.positionWeight.changeFrame(referenceFrame);
   }

   private void setPositionInternal(FramePoint3D desiredPosition)
   {
      this.position.setIncludingFrame(desiredPosition);
      this.position.changeFrame(referenceFrame);
   }

   private void setPositionMinInternal(FramePoint3D minPosition)
   {
      this.positionMin.setIncludingFrame(minPosition);
      this.positionMin.changeFrame(referenceFrame);
   }

   private void setPositionMaxInternal(FramePoint3D maxPosition)
   {
      this.positionMax.setIncludingFrame(maxPosition);
      this.positionMax.changeFrame(referenceFrame);
   }

   public void setOrientationObjective(FrameQuaternion desiredOrienation, FrameVector3D orientationWeight)
   {
      setOrientationInternal(desiredOrienation);
      this.orientationConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setOrientationWeightInternal(orientationWeight);
   }

   private void setOrientationWeightInternal(FrameVector3D orientationWeight)
   {
      this.orientationWeight.setIncludingFrame(orientationWeight);
      this.orientationWeight.changeFrame(referenceFrame);
   }

   private void setOrientationInternal(FrameQuaternion desiredOrienation)
   {
      this.orientation.setIncludingFrame(desiredOrienation);
      this.orientation.changeFrame(referenceFrame);
   }

   public void setOrientationConstraint(FrameQuaternion desiredOrienation)
   {
      setOrientationInternal(desiredOrienation);
      this.orientationConstraintType.setXYZ(DependentVariableConstraintType.EQUALITY);
      this.orientationWeight.setToNaN();
   }

   public void setOrientation(FrameQuaternion desiredOrienation, VectorEnum<DependentVariableConstraintType> constraintType, FrameVector3D orientationWeight)
   {
      setOrientationInternal(desiredOrienation);
      this.orientationConstraintType.set(constraintType);
      setOrientationWeightInternal(orientationWeight);
   }

   public void setLinearVelocityObjective(FrameVector3D desiredLinearVelocity, FrameVector3D linearVelocityWeight)
   {
      setLinearVelocityInternal(desiredLinearVelocity);
      this.linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setLinearVelocityWeightInternal(linearVelocityWeight);
      this.linearVelocityMax.setToNaN();
      this.linearVelocityMin.setToNaN();
   }

   public void setLinearVelocityInequalitiesForObjective(FrameVector3D maxLinearVelocity, FrameVector3D minLinearVelocity)
   {
      setLinearVelocityMaxInternal(maxLinearVelocity);
      setLinearVelocityMinInternal(minLinearVelocity);
   }

   public void setLinearVelocityInequalities(FrameVector3D maxLinearVelocity, FrameVector3D minLinearVelocity)
   {
      this.linearVelocity.setToNaN();
      this.linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      this.linearVelocityWeight.setToNaN();
      setLinearVelocityMaxInternal(maxLinearVelocity);
      setLinearVelocityMinInternal(minLinearVelocity);
   }

   public void setLinearVelocityObjective(FrameVector3D desiredLinearVelocity, FrameVector3D linearVelocityWeight, FrameVector3D maxLinearVelocity,
                                          FrameVector3D minLinearVelocity)
   {
      setLinearVelocityInternal(desiredLinearVelocity);
      this.linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setLinearVelocityWeightInternal(linearVelocityWeight);
      setLinearVelocityMaxInternal(maxLinearVelocity);
      setLinearVelocityMinInternal(minLinearVelocity);
   }

   private void setLinearVelocityWeightInternal(FrameVector3D linearVelocityWeight)
   {
      this.linearVelocityWeight.setIncludingFrame(linearVelocityWeight);
      this.linearVelocityWeight.changeFrame(referenceFrame);
   }

   private void setLinearVelocityMinInternal(FrameVector3D minLinearVelocity)
   {
      this.linearVelocityMin.setIncludingFrame(minLinearVelocity);
      this.linearVelocityMin.changeFrame(referenceFrame);
   }

   private void setLinearVelocityMaxInternal(FrameVector3D maxLinearVelocity)
   {
      this.linearVelocityMax.setIncludingFrame(maxLinearVelocity);
      this.linearVelocityMax.changeFrame(referenceFrame);
   }

   public void setLinearVelocityConstraint(FrameVector3D desiredLinearVelocity)
   {
      setLinearVelocityInternal(desiredLinearVelocity);
      this.linearVelocityConstraintType.setXYZ(DependentVariableConstraintType.EQUALITY);
      this.linearVelocityWeight.setToNaN();
      this.linearVelocityMax.setToNaN();
      this.linearVelocityMin.setToNaN();
   }

   private void setLinearVelocityInternal(FrameVector3D desiredLinearVelocity)
   {
      this.linearVelocity.setIncludingFrame(desiredLinearVelocity);
      this.linearVelocity.changeFrame(referenceFrame);
   }

   public void setLinearVelocityObjective(FrameVector3D desiredLinearVelocity, VectorEnum<DependentVariableConstraintType> constraintType,
                                          FrameVector3D linearVelocityWeight, FrameVector3D maxLinearVelocity, FrameVector3D minLinearVelocity)
   {
      setLinearVelocityInternal(desiredLinearVelocity);
      this.linearVelocityConstraintType.set(constraintType);
      setLinearVelocityWeightInternal(linearVelocityWeight);
      setLinearVelocityMaxInternal(maxLinearVelocity);
      setLinearVelocityMinInternal(minLinearVelocity);
   }

   public void setAngularVelocityObjective(FrameVector3D desiredAngularVelocity, FrameVector3D angularVelocityWeight)
   {
      setAngularVelocityInternal(desiredAngularVelocity);
      this.angularVelocityConstraintType.setXYZ(DependentVariableConstraintType.OBJECTIVE);
      setAngularVelocityWeightInternal(angularVelocityWeight);
   }

   private void setAngularVelocityWeightInternal(FrameVector3D angularVelocityWeight)
   {
      this.angularVelocityWeight.setIncludingFrame(angularVelocityWeight);
      this.angularVelocityWeight.changeFrame(referenceFrame);
   }

   private void setAngularVelocityInternal(FrameVector3D desiredAngularVelocity)
   {
      this.angularVelocity.setIncludingFrame(desiredAngularVelocity);
      this.angularVelocity.changeFrame(referenceFrame);
   }

   public void setAngularVelocityConstraint(FrameVector3D desiredAngularVelocity)
   {
      setAngularVelocityInternal(desiredAngularVelocity);
      this.angularVelocityConstraintType.setXYZ(DependentVariableConstraintType.EQUALITY);
      this.angularVelocityWeight.setToNaN();
   }

   public void setAngularVelocity(FrameVector3D desiredAngularVelocity, VectorEnum<DependentVariableConstraintType> constraintType,
                                  FrameVector3D angularVelocityWeight)
   {
      setAngularVelocityInternal(desiredAngularVelocity);
      this.angularVelocityConstraintType.set(constraintType);
      setAngularVelocityWeightInternal(angularVelocityWeight);
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
      this.positionMax.setIncludingFrame(other.positionMax);
      this.positionMin.setIncludingFrame(other.positionMin);

      this.linearVelocity.setIncludingFrame(other.linearVelocity);
      this.linearVelocityWeight.setIncludingFrame(other.linearVelocityWeight);
      this.linearVelocityConstraintType.set(other.linearVelocityConstraintType);
      this.linearVelocityMax.setIncludingFrame(other.linearVelocityMax);
      this.linearVelocityMin.setIncludingFrame(other.linearVelocityMin);

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

   public double getPositionMax(Axis axis)
   {
      return positionMax.getElement(axis.ordinal());
   }

   public double getPositionMin(Axis axis)
   {
      return positionMin.getElement(axis.ordinal());
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

   public double getLinearVelocityMax(Axis axis)
   {
      return linearVelocityMax.getElement(axis.ordinal());
   }

   public double getLinearVelocityMin(Axis axis)
   {
      return linearVelocityMin.getElement(axis.ordinal());
   }

   public DependentVariableConstraintType getLinearVelocityConstraintType(Axis axis)
   {
      return linearVelocityConstraintType.getElement(axis);
   }

   public double getOrientationValue(Axis axis)
   {
      return orientation.getElement(axis.ordinal());
   }

   public double getOrientationWeight(Axis axis)
   {
      return orientationWeight.getElement(axis.ordinal());
   }

   public DependentVariableConstraintType getOrientationConstraintType(Axis axis)
   {
      return orientationConstraintType.getElement(axis);
   }

   public double getAngularVelocityValue(Axis axis)
   {
      return angularVelocity.getElement(axis.ordinal());
   }

   public double getAngularVelocityWeight(Axis axis)
   {
      return angularVelocityWeight.getElement(axis.ordinal());
   }

   public DependentVariableConstraintType getAngularVelocityConstraintType(Axis axis)
   {
      return angularVelocityConstraintType.getElement(axis);
   }

   public double getTorqueValue(Axis axis)
   {
      return torque.getElement(axis.ordinal());
   }

   public double getTorqueWeight(Axis axis)
   {
      return torqueWeight.getElement(axis.ordinal());
   }

   public DependentVariableConstraintType getTorqueConstraintType(Axis axis)
   {
      return torqueConstraintType.getElement(axis);
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

   public void getForce(FrameVector3D forceToPack)
   {
      forceToPack.setIncludingFrame(force);
   }

   public void getForceRate(FrameVector3D forceRateToPack)
   {
      forceRateToPack.setIncludingFrame(forceRate);
   }

   public String toString()
   {
      return "Max position: " + this.positionMax.toString() + ", Min position: " + this.positionMin.toString();
   }
}