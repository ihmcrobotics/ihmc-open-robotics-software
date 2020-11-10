package us.ihmc.robotics.screwTheory;

import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.robotics.kinematics.fourbar.FourBar;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class InvertedFourBarJoint implements OneDoFJointBasics
{
   private final String name;
   private final String nameId;
   private final RigidBodyBasics predecessor;
   private RigidBodyBasics successor;
   private final MovingReferenceFrame beforeJointFrame;
   private final MovingReferenceFrame afterJointFrame;

   private final FourBarKinematicLoopFunction fourBarFunction;
   private InvertedFourBarJointIKSolver ikSolver;

   private final TwistReadOnly jointTwist;
   private final Twist unitJointTwist = new Twist();
   private final Twist unitSuccessorTwist = new Twist();
   private final Twist unitPredecessorTwist = new Twist();
   private final List<TwistReadOnly> unitTwists;

   private final SpatialAccelerationReadOnly jointAcceleration;
   private final SpatialAcceleration jointBiasAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration successorBiasAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration unitJointAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration unitSuccessorAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration unitPredecessorAcceleration = new SpatialAcceleration();

   private final Wrench unitJointWrench = new Wrench();
   private WrenchReadOnly jointWrench;

   /** Variable to store intermediate results for garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();

   /**
    * Creates a new inverted four bar joint that is to wrap the 4 given revolute joints into a single
    * 1-DoF joint.
    * <p>
    * <b>WARNING: This joint is somewhat tricky to create, as the 4 given revolute joints are only used
    * as a template to setup this complex joint and for internal calculation.</b><br>
    * Here are the expected construction steps of a robot system:
    * <ol>
    * <li>The user should create the branch of the robot up to the 4 revolute joints composing the four
    * bar.
    * <li>Instead of adding the successor to the last 2 joints, create a dummy rigid-body to terminate
    * the four bar.
    * <li>Create the {@code InvertedFourBarJoint} given the four joints.
    * <li>Finally proceed to creating the subtree following the four bar by attaching the next
    * successor to this new four bar joint. The transform, a.k.a. inertia pose, that is to be provided
    * to the successor should expressed with respect to {@link #getJointD()}'s frame after joint.
    * </ol>
    * </p>
    *
    * @param name             the name of this joint.
    * @param fourBarJoints    the 4 revolute joints composing the four bar.
    * @param masterJointIndex the index in {@code fourBarJoints} of the joints that is actuated.
    * @throws IllegalArgumentException if the given joints do not represent an inverted four bar
    *                                  joints.
    * @throws IllegalArgumentException if a subtree is already attached to the last two joints closing
    *                                  the four bar.
    * @see FourBarKinematicLoopFunctionTools#configureFourBarKinematics(RevoluteJointBasics[],
    *      FourBarToJointConverter[], FourBar, int, double)
    */
   public InvertedFourBarJoint(String name, RevoluteJointBasics[] fourBarJoints, int masterJointIndex)
   {
      fourBarFunction = new FourBarKinematicLoopFunction(name, fourBarJoints, masterJointIndex);
      if (!fourBarFunction.isInverted())
         throw new IllegalArgumentException("The given joint configuration does not represent an inverted four bar.");
      setIKSolver(new InvertedFourBarJointIKBinarySolver(1.0e-5));

      this.name = name;
      predecessor = getJointA().getPredecessor();
      // Detaching the joints A & B from the predecessor and attaching this joint.
      predecessor.getChildrenJoints().remove(getJointA());
      predecessor.getChildrenJoints().remove(getJointB());
      predecessor.addChildJoint(this);

      beforeJointFrame = getJointA().getFrameBeforeJoint();
      afterJointFrame = getJointD().getFrameAfterJoint();

      if (predecessor.isRootBody())
         nameId = name;
      else
         nameId = predecessor.getParentJoint().getNameId() + NAME_ID_SEPARATOR + name;

      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      jointAcceleration = setupJointAcceleration();
   }

   private SpatialAccelerationReadOnly setupJointAcceleration()
   {
      DoubleSupplier wx = () -> getQdd() * unitJointAcceleration.getAngularPartX() + jointBiasAcceleration.getAngularPartX();
      DoubleSupplier wy = () -> getQdd() * unitJointAcceleration.getAngularPartY() + jointBiasAcceleration.getAngularPartY();
      DoubleSupplier wz = () -> getQdd() * unitJointAcceleration.getAngularPartZ() + jointBiasAcceleration.getAngularPartZ();
      DoubleSupplier vx = () -> getQdd() * unitJointAcceleration.getLinearPartX() + jointBiasAcceleration.getLinearPartX();
      DoubleSupplier vy = () -> getQdd() * unitJointAcceleration.getLinearPartY() + jointBiasAcceleration.getLinearPartY();
      DoubleSupplier vz = () -> getQdd() * unitJointAcceleration.getLinearPartZ() + jointBiasAcceleration.getLinearPartZ();
      FrameVector3DReadOnly angularPart = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this::getFrameAfterJoint, wx, wy, wz);
      FrameVector3DReadOnly linearPart = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this::getFrameAfterJoint, vx, vy, vz);
      return MecanoFactories.newSpatialAccelerationVectorReadOnly(afterJointFrame, beforeJointFrame, angularPart, linearPart);
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      if (this.successor != null)
         throw new IllegalOperationException("The successor of this joint has already been set.");

      this.successor = successor;
      jointWrench = MecanoFactories.newWrenchReadOnly(this::getTau, unitJointWrench);
   }

   /**
    * Sets the solver to use for computing the four bar configuration given the joint angle via
    * {@link #setQ(double)}.
    *
    * @param ikSolver the solver to use.
    */
   public void setIKSolver(InvertedFourBarJointIKSolver ikSolver)
   {
      this.ikSolver = ikSolver;
      ikSolver.setConverters(fourBarFunction.getConverters());
   }

   @Override
   public void updateFramesRecursively()
   {
      fourBarFunction.updateState(true, true);
      getJointA().getFrameBeforeJoint().update();
      getJointB().getFrameBeforeJoint().update();
      getJointC().getFrameBeforeJoint().update();
      getJointD().getFrameBeforeJoint().update();
      getJointA().getFrameAfterJoint().update();
      getJointB().getFrameAfterJoint().update();
      getJointC().getFrameAfterJoint().update();
      getJointD().getFrameAfterJoint().update();

      updateMotionSubspace();

      if (getSuccessor() != null)
      {
         getSuccessor().updateFramesRecursively();
      }
   }

   private final Twist tempTwist = new Twist();
   private final SpatialAcceleration tempAcceleration = new SpatialAcceleration();
   private final Twist deltaTwist = new Twist();
   private final Twist bodyTwist = new Twist();

   @Override
   public void updateMotionSubspace()
   {
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      DMatrixRMaj loopConvectiveTerm = fourBarFunction.getLoopConvectiveTerm();
      RevoluteJointBasics jointA = getJointA();
      RevoluteJointBasics jointD = getJointD();
      double J_A = loopJacobian.get(0);
      double J_D = loopJacobian.get(3);

      unitJointTwist.setIncludingFrame(jointA.getUnitJointTwist());
      unitJointTwist.scale(J_A);
      unitJointTwist.setBodyFrame(jointD.getFrameBeforeJoint());
      unitJointTwist.changeFrame(jointD.getFrameAfterJoint());
      tempTwist.setIncludingFrame(jointD.getUnitJointTwist());
      tempTwist.scale(J_D);
      unitJointTwist.add(tempTwist);
      unitJointTwist.scale(1.0 / (J_A + J_D));
      // Since we're ignoring the bias terms, the unit-accelerations are the same as the unit-twists.
      unitJointAcceleration.setIncludingFrame(unitJointTwist);

      /*
       * This next block is for computing the bias acceleration. I ended up using tests to figure out
       * exactly what it should, but I feel that it can be simplified.
       */
      jointD.getFrameAfterJoint().getTwistRelativeToOther(jointA.getFrameAfterJoint(), deltaTwist);
      jointD.getFrameBeforeJoint().getTwistRelativeToOther(jointA.getFrameBeforeJoint(), bodyTwist);
      deltaTwist.changeFrame(jointD.getFrameAfterJoint());
      bodyTwist.changeFrame(jointD.getFrameAfterJoint());
      jointBiasAcceleration.setIncludingFrame(jointA.getUnitJointAcceleration());
      jointBiasAcceleration.scale(loopConvectiveTerm.get(0));
      jointBiasAcceleration.setBodyFrame(jointD.getFrameBeforeJoint());
      jointBiasAcceleration.changeFrame(jointD.getFrameAfterJoint(), deltaTwist, bodyTwist);
      tempAcceleration.setIncludingFrame(jointD.getUnitJointAcceleration());
      tempAcceleration.scale(loopConvectiveTerm.get(3));
      jointBiasAcceleration.add(tempAcceleration);
      tempAcceleration.setIncludingFrame(unitJointAcceleration);
      tempAcceleration.scale(-(loopConvectiveTerm.get(0) + loopConvectiveTerm.get(3)));
      jointBiasAcceleration.add((SpatialVectorReadOnly) tempAcceleration);

      if (getSuccessor() != null)
      {
         unitSuccessorTwist.setIncludingFrame(unitJointTwist);
         unitSuccessorTwist.setBaseFrame(predecessor.getBodyFixedFrame());
         unitSuccessorTwist.setBodyFrame(successor.getBodyFixedFrame());
         unitSuccessorTwist.changeFrame(successor.getBodyFixedFrame());

         unitPredecessorTwist.setIncludingFrame(unitSuccessorTwist);
         unitPredecessorTwist.invert();
         unitPredecessorTwist.changeFrame(predecessor.getBodyFixedFrame());

         // Since we're ignoring the bias terms, the unit-accelerations are the same as the unit-twists.
         unitSuccessorAcceleration.setIncludingFrame(unitSuccessorTwist);
         unitPredecessorAcceleration.setIncludingFrame(unitPredecessorTwist);

         successorBiasAcceleration.setIncludingFrame(jointBiasAcceleration);
         successorBiasAcceleration.setBaseFrame(getPredecessor().getBodyFixedFrame());
         successorBiasAcceleration.setBodyFrame(getSuccessor().getBodyFixedFrame());
         successorBiasAcceleration.changeFrame(getSuccessor().getBodyFixedFrame());

         unitJointWrench.setIncludingFrame(fourBarFunction.getMasterJoint().getUnitJointTwist());
         unitJointWrench.changeFrame(afterJointFrame);
         unitJointWrench.setBodyFrame(getSuccessor().getBodyFixedFrame());
      }
   }

   public FourBarKinematicLoopFunction getFourBarFunction()
   {
      return fourBarFunction;
   }

   public RevoluteJointBasics getMasterJoint()
   {
      return fourBarFunction.getMasterJoint();
   }

   public RevoluteJointBasics getJointA()
   {
      return fourBarFunction.getJointA();
   }

   public RevoluteJointBasics getJointB()
   {
      return fourBarFunction.getJointB();
   }

   public RevoluteJointBasics getJointC()
   {
      return fourBarFunction.getJointC();
   }

   public RevoluteJointBasics getJointD()
   {
      return fourBarFunction.getJointD();
   }

   public InvertedFourBarJointIKSolver getIKSolver()
   {
      return ikSolver;
   }

   @Override
   public MovingReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   @Override
   public MovingReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   @Override
   public RigidBodyBasics getPredecessor()
   {
      return predecessor;
   }

   @Override
   public RigidBodyBasics getSuccessor()
   {
      return successor;
   }

   @Override
   public boolean isMotionSubspaceVariable()
   {
      return true;
   }

   @Override
   public MovingReferenceFrame getLoopClosureFrame()
   {
      return null;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getNameId()
   {
      return nameId;
   }

   @Override
   public void setupLoopClosure(RigidBodyBasics successor, RigidBodyTransformReadOnly transformFromSuccessorParentJoint)
   {
      throw new UnsupportedOperationException("Loop closure using a four bar joint has not been implemented.");
   }

   @Override
   public FrameVector3DReadOnly getJointAxis()
   {
      return getMasterJoint().getJointAxis();
   }

   @Override
   public double getQ()
   {
      return getJointA().getQ() + getJointD().getQ();
   }

   @Override
   public double getQd()
   {
      return getJointA().getQd() + getJointD().getQd();
   }

   @Override
   public double getQdd()
   {
      return getJointA().getQdd() + getJointD().getQdd();
   }

   @Override
   public double getTau()
   {
      // TODO This method ignores potentially non-zero torques set in the other joints.
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      fourBarFunction.updateEffort();
      if (getMasterJoint() == getJointA() || getMasterJoint() == getJointD())
         return getMasterJoint().getTau() / (loopJacobian.get(0) + loopJacobian.get(3));
      else
         return getMasterJoint().getTau() / (loopJacobian.get(1) + loopJacobian.get(2));
   }

   @Override
   public double getJointLimitLower()
   {
      return getJointA().getJointLimitLower() + getJointD().getJointLimitLower();
   }

   @Override
   public double getJointLimitUpper()
   {
      return getJointA().getJointLimitUpper() + getJointD().getJointLimitUpper();
   }

   @Override
   public double getVelocityLimitLower()
   {
      return getJointA().getVelocityLimitLower() + getJointD().getVelocityLimitLower();
   }

   @Override
   public double getVelocityLimitUpper()
   {
      return getJointA().getVelocityLimitUpper() + getJointD().getVelocityLimitUpper();
   }

   @Override
   public double getEffortLimitLower()
   {
      return getMasterJoint().getEffortLimitLower();
   }

   @Override
   public double getEffortLimitUpper()
   {
      return getMasterJoint().getEffortLimitUpper();
   }

   @Override
   public TwistReadOnly getUnitJointTwist()
   {
      return unitJointTwist;
   }

   @Override
   public TwistReadOnly getUnitSuccessorTwist()
   {
      return unitSuccessorTwist;
   }

   @Override
   public TwistReadOnly getUnitPredecessorTwist()
   {
      return unitPredecessorTwist;
   }

   @Override
   public SpatialAccelerationReadOnly getUnitJointAcceleration()
   {
      return unitJointAcceleration;
   }

   @Override
   public SpatialAccelerationReadOnly getUnitSuccessorAcceleration()
   {
      return unitSuccessorAcceleration;
   }

   @Override
   public SpatialAccelerationReadOnly getUnitPredecessorAcceleration()
   {
      return unitPredecessorAcceleration;
   }

   @Override
   public void getJointConfiguration(RigidBodyTransform jointConfigurationToPack)
   {
      afterJointFrame.getTransformToDesiredFrame(jointConfigurationToPack, beforeJointFrame);
   }

   @Override
   public TwistReadOnly getJointTwist()
   {
      return jointTwist;
   }

   @Override
   public List<TwistReadOnly> getUnitTwists()
   {
      return unitTwists;
   }

   @Override
   public SpatialAccelerationReadOnly getJointAcceleration()
   {
      return jointAcceleration;
   }

   @Override
   public SpatialAccelerationReadOnly getJointBiasAcceleration()
   {
      return jointBiasAcceleration;
   }

   @Override
   public SpatialAccelerationReadOnly getSuccessorBiasAcceleration()
   {
      return successorBiasAcceleration;
   }

   @Override
   public void getPredecessorAcceleration(SpatialAccelerationBasics accelerationToPack)
   {
      // OneDoFJointReadOnly.getPredecessorAcceleration(...) was not used when creating this joint.
      // Implementing it would require extra calculation in the updateMotionSubspace().
      throw new UnsupportedOperationException("Implement me!");
   }

   @Override
   public SpatialAccelerationReadOnly getPredecessorBiasAcceleration()
   {
      // OneDoFJointReadOnly.getPredecessorBiasAcceleration() was not used when creating this joint.
      // Implementing it would require extra calculation in the updateMotionSubspace().
      throw new UnsupportedOperationException("Implement me!");
   }

   @Override
   public WrenchReadOnly getJointWrench()
   {
      return jointWrench;
   }

   @Override
   public void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      jointOrientation.getRotationVector(rotationVector);
      setQ(rotationVector.dot(getJointAxis()));
   }

   @Override
   public void setJointPosition(Tuple3DReadOnly jointPosition)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   public void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      setQd(jointAngularVelocity.dot(getJointAxis()));
   }

   @Override
   public void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   public void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      setQdd(jointAngularAcceleration.dot(getJointAxis()));
   }

   @Override
   public void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   public void setJointTorque(Vector3DReadOnly jointTorque)
   {
      setTau(jointTorque.dot(getJointAxis()));
   }

   @Override
   public void setJointForce(Vector3DReadOnly jointForce)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   public void setQ(double q)
   {
      getMasterJoint().setQ(ikSolver.solve(q, fourBarFunction.getMasterVertex()));
   }

   @Override
   public void setQd(double qd)
   {
      fourBarFunction.updateState(false, false);
      // qd = (J_A + J_D) qd_M = (J_B + J_C) qd_M
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      double qd_master = qd / (loopJacobian.get(0) + loopJacobian.get(3));
      getMasterJoint().setQd(qd_master);
   }

   @Override
   public void setQdd(double qdd)
   {
      fourBarFunction.updateState(false, false);
      // qdd = (J_A + J_D) qdd_M + c_A + c_D = (J_B + J_C) qdd_M + c_B + c_C
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      DMatrixRMaj loopConvectiveTerm = fourBarFunction.getLoopConvectiveTerm();
      qdd = qdd - loopConvectiveTerm.get(0) - loopConvectiveTerm.get(3);
      double qdd_master = qdd / (loopJacobian.get(0) + loopJacobian.get(3));
      getMasterJoint().setQdd(qdd_master);
   }

   @Override
   public void setTau(double tau)
   {
      getJointA().setJointTauToZero();
      getJointB().setJointTauToZero();
      getJointC().setJointTauToZero();
      getJointD().setJointTauToZero();

      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      if (getMasterJoint() == getJointA() || getMasterJoint() == getJointD())
         getMasterJoint().setTau((loopJacobian.get(0) + loopJacobian.get(3)) * tau);
      else
         getMasterJoint().setTau((loopJacobian.get(1) + loopJacobian.get(2)) * tau);
   }

   @Override
   public void setJointLimitLower(double jointLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void setJointLimitUpper(double jointLimitUpper)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void setVelocityLimitLower(double velocityLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void setVelocityLimitUpper(double velocityLimitUpper)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void setEffortLimitLower(double effortLimitLower)
   {
      getMasterJoint().setEffortLimitLower(effortLimitLower);
   }

   @Override
   public void setEffortLimitUpper(double effortLimitUpper)
   {
      getMasterJoint().setEffortLimitUpper(effortLimitUpper);
   }

   /**
    * Returns the implementation name of this joint and the joint name.
    */
   @Override
   public String toString()
   {
      String qAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQ());
      String qdAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQd());
      String qddAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQdd());
      String tauAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getTau());
      return getClass().getSimpleName() + " " + getName() + ", q: " + qAsString + ", qd: " + qdAsString + ", qdd: " + qddAsString + ", tau: " + tauAsString;
   }

   /**
    * The hash code of a joint is based on its {@link #getNameId()}.
    *
    * @return the hash code of the {@link #getNameId()} of this joint.
    */
   @Override
   public int hashCode()
   {
      return nameId.hashCode();
   }
}
