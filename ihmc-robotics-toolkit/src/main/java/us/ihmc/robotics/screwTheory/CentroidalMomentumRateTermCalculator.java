package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * The centroidal momentum can be written as \dot{h} = A * \dot{v} + \dot{A} * v. This class
 * calculates both the A matrix and the \dot{A} * v term. This implementation is going for
 * speed/efficiency, so the readability is not the best. For a more readable implementation, see
 * CentroidalMomentumMatrix for the A matrix and CentroidalMomentumRateADotVTerm for the \dot{A} * v
 * term.
 */

public class CentroidalMomentumRateTermCalculator implements ReferenceFrameHolder
{
   private final ReferenceFrame matrixFrame;
   private final InverseDynamicsJoint[] joints;
   private final RecursionStep initialRecursionStep;
   private final Map<RigidBody, RecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();

   private final DenseMatrix64F centroidalMomentumMatrix;
   private final SpatialForceVector biasSpatialForce = new SpatialForceVector();
   private final DenseMatrix64F biasSpatialForceMatrix = new DenseMatrix64F(6, 1);

   private final DenseMatrix64F jointVelocityMatrix;
   private final DenseMatrix64F jointAccelerationMatrix;
   private final DenseMatrix64F momentumMatrix = new DenseMatrix64F(6, 1);

   private final Momentum momentum;
   private final SpatialForceVector momentumRate;
   private double totalMass = 0.0;
   private final FixedFrameVector3DBasics centerOfMassVelocity;
   private final FixedFrameVector3DBasics centerOfMassAcceleration;

   private boolean isCentroidalMomentumUpToDate = false;
   private boolean isCentroidalMomentumMatrixUpToDate = false;
   private boolean isJointVelocityMatrixUpToDate = false;
   private boolean isJointAccelerationMatrixUpToDate = false;
   private boolean isMomentumUpToDate = false;
   private boolean isMomentumRateUpToDate = false;
   private boolean isTotalMassUpToDate = false;
   private boolean isCenterOfMassVelocityUpToDate = false;
   private boolean isCenterOfMassAccelerationUpToDate = false;

   public CentroidalMomentumRateTermCalculator(RigidBody rootBody, ReferenceFrame matrixFrame)
   {
      this(ScrewTools.computeSubtreeJoints(rootBody), matrixFrame);
   }

   public CentroidalMomentumRateTermCalculator(List<? extends InverseDynamicsJoint> jointsToConsider, ReferenceFrame matrixFrame)
   {
      this(jointsToConsider.toArray(new InverseDynamicsJoint[jointsToConsider.size()]), matrixFrame);
   }

   public CentroidalMomentumRateTermCalculator(InverseDynamicsJoint[] jointsToConsider, ReferenceFrame matrixFrame)
   {
      this.matrixFrame = matrixFrame;

      RigidBody rootBody = ScrewTools.getRootBody(jointsToConsider[0].getPredecessor());
      initialRecursionStep = new RecursionStep(rootBody, null, matrixFrame);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      joints = jointsToConsider;
      buildMultiBodyTree(initialRecursionStep, new HashSet<>(Arrays.asList(jointsToConsider)));

      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(joints);
      this.centroidalMomentumMatrix = new DenseMatrix64F(6, nDegreesOfFreedom);
      jointVelocityMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
      jointAccelerationMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);

      momentum = new Momentum(matrixFrame);
      momentumRate = new SpatialForceVector(matrixFrame);
      centerOfMassVelocity = new FrameVector3D(matrixFrame);
      centerOfMassAcceleration = new FrameVector3D(matrixFrame);
   }

   private void buildMultiBodyTree(RecursionStep parent, Collection<? extends InverseDynamicsJoint> jointsToConsider)
   {
      for (InverseDynamicsJoint childJoint : parent.rigidBody.getChildrenJoints())
      {
         if (!jointsToConsider.contains(childJoint))
            continue;

         RigidBody childBody = childJoint.getSuccessor();
         if (childBody != null)
         {
            RecursionStep child = new RecursionStep(childBody, parent, matrixFrame);
            rigidBodyToRecursionStepMap.put(childBody, child);
            parent.children.add(child);
            buildMultiBodyTree(child, jointsToConsider);
         }
      }
   }

   public void reset()
   {
      isCentroidalMomentumUpToDate = false;
      isCentroidalMomentumMatrixUpToDate = false;
      isJointVelocityMatrixUpToDate = false;
      isJointAccelerationMatrixUpToDate = false;
      isMomentumUpToDate = false;
      isMomentumRateUpToDate = false;
      isTotalMassUpToDate = false;
      isCenterOfMassVelocityUpToDate = false;
      isCenterOfMassAccelerationUpToDate = false;
   }

   private void updateCentroidalMomentum()
   {
      if (isCentroidalMomentumUpToDate)
         return;

      passOne(initialRecursionStep);
      passTwo(initialRecursionStep);
      isCentroidalMomentumUpToDate = true;
   }

   private void passOne(RecursionStep current)
   {
      current.passOne();

      for (int childIndex = 0; childIndex < current.children.size(); childIndex++)
      {
         RecursionStep child = current.children.get(childIndex);
         passOne(child);
      }
   }

   private void passTwo(RecursionStep current)
   {
      current.passTwo();

      for (int childIndex = 0; childIndex < current.children.size(); childIndex++)
      {
         RecursionStep child = current.children.get(childIndex);
         passTwo(child);
      }
   }

   private void updateCentroidalMomentumMatrixAndBiasForce()
   {
      if (isCentroidalMomentumMatrixUpToDate)
         return;

      updateCentroidalMomentum();
      int columnIndex = 0;

      biasSpatialForce.setToZero(matrixFrame);

      for (InverseDynamicsJoint joint : joints)
      {
         RecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

         CommonOps.insert(recursionStep.centroidalMomentumMatrixBlock, centroidalMomentumMatrix, 0, columnIndex);

         columnIndex += joint.getDegreesOfFreedom();

         biasSpatialForce.add(recursionStep.biasWrench);
      }

      biasSpatialForce.getMatrix(biasSpatialForceMatrix);

      isCentroidalMomentumMatrixUpToDate = true;
   }

   private DenseMatrix64F getJointVelocityMatrix()
   {
      if (!isJointVelocityMatrixUpToDate)
      {
         ScrewTools.getJointVelocitiesMatrix(joints, jointVelocityMatrix);
         isJointVelocityMatrixUpToDate = true;
      }

      return jointVelocityMatrix;
   }

   private DenseMatrix64F getJointAccelerationMatrix()
   {
      if (!isJointAccelerationMatrixUpToDate)
      {
         ScrewTools.getDesiredJointAccelerationsMatrix(joints, jointAccelerationMatrix);
         isJointAccelerationMatrixUpToDate = true;
      }

      return jointAccelerationMatrix;
   }

   public DenseMatrix64F getCentroidalMomentumMatrix()
   {
      updateCentroidalMomentumMatrixAndBiasForce();
      return centroidalMomentumMatrix;
   }

   public DenseMatrix64F getCentroidalMomentumMatrix(InverseDynamicsJoint[] joints)
   {
      DenseMatrix64F centroidalMomentumMatrix = new DenseMatrix64F(6, ScrewTools.computeDegreesOfFreedom(joints));
      getCentroidalMomentumMatrix(joints, centroidalMomentumMatrix);
      return centroidalMomentumMatrix;
   }

   public void getCentroidalMomentumMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F centroidalMomentumMatrixToPack)
   {
      if (joints.length != this.joints.length)
         throw new IllegalArgumentException("Incompatible array of joints.");

      updateCentroidalMomentum();

      int numberOfColumns = ScrewTools.computeDegreesOfFreedom(joints);
      centroidalMomentumMatrixToPack.reshape(6, numberOfColumns);

      int columnIndex = 0;

      for (InverseDynamicsJoint joint : joints)
      {
         RigidBody successor = joint.getSuccessor();
         RecursionStep recursionStep = rigidBodyToRecursionStepMap.get(successor);

         CommonOps.insert(recursionStep.centroidalMomentumMatrixBlock, centroidalMomentumMatrixToPack, 0, columnIndex);

         columnIndex += joint.getDegreesOfFreedom();
      }
   }

   public DenseMatrix64F getCentroidalMomentumMatrix(List<? extends InverseDynamicsJoint> joints)
   {
      DenseMatrix64F centroidalMomentumMatrix = new DenseMatrix64F(6, ScrewTools.computeDegreesOfFreedom(joints));
      getCentroidalMomentumMatrix(joints, centroidalMomentumMatrix);
      return centroidalMomentumMatrix;
   }

   public void getCentroidalMomentumMatrix(List<? extends InverseDynamicsJoint> joints, DenseMatrix64F centroidalMomentumMatrixToPack)
   {
      updateCentroidalMomentum();

      int numberOfColumns = ScrewTools.computeDegreesOfFreedom(joints);
      centroidalMomentumMatrixToPack.reshape(6, numberOfColumns);

      int columnIndex = 0;

      for (InverseDynamicsJoint joint : joints)
      {
         RigidBody successor = joint.getSuccessor();
         RecursionStep recursionStep = rigidBodyToRecursionStepMap.get(successor);

         CommonOps.insert(recursionStep.centroidalMomentumMatrixBlock, centroidalMomentumMatrixToPack, 0, columnIndex);

         columnIndex += joint.getDegreesOfFreedom();
      }
   }

   public SpatialForceVector getBiasSpatialForce()
   {
      updateCentroidalMomentumMatrixAndBiasForce();
      return biasSpatialForce;
   }

   public DenseMatrix64F getBiasSpatialForceMatrix()
   {
      updateCentroidalMomentumMatrixAndBiasForce();
      return biasSpatialForceMatrix;
   }

   public Momentum getMomentum()
   {
      if (!isMomentumUpToDate)
      {
         CommonOps.mult(getCentroidalMomentumMatrix(), getJointVelocityMatrix(), momentumMatrix);
         momentum.angularPart.set(0, momentumMatrix);
         momentum.linearPart.set(3, momentumMatrix);
         isMomentumUpToDate = true;
      }
      return momentum;
   }

   public SpatialForceVector getMomentumRate()
   {
      if (!isMomentumRateUpToDate)
      {
         CommonOps.mult(getCentroidalMomentumMatrix(), getJointAccelerationMatrix(), momentumMatrix);
         momentumRate.angularPart.set(0, momentumMatrix);
         momentumRate.linearPart.set(3, momentumMatrix);
         momentumRate.add(getBiasSpatialForce());
         isMomentumRateUpToDate = true;
      }
      return momentumRate;
   }

   public void getMomentumRate(DenseMatrix64F jointAccelerationMatrix, SpatialForceVector momentumRateToPack)
   {
      CommonOps.mult(getCentroidalMomentumMatrix(), jointAccelerationMatrix, momentumMatrix);
      momentumRateToPack.setToZero(matrixFrame);
      momentumRateToPack.angularPart.set(0, momentumMatrix);
      momentumRateToPack.linearPart.set(3, momentumMatrix);
      momentumRateToPack.add(getBiasSpatialForce());
   }

   public void getMomentumRate(InverseDynamicsJoint[] joints, DenseMatrix64F jointAccelerationMatrix, SpatialForceVector momentumRateToPack)
   {
      CommonOps.mult(getCentroidalMomentumMatrix(joints), jointAccelerationMatrix, momentumMatrix);
      momentumRateToPack.setToZero(matrixFrame);
      momentumRateToPack.angularPart.set(0, momentumMatrix);
      momentumRateToPack.linearPart.set(3, momentumMatrix);
      momentumRateToPack.add(getBiasSpatialForce());
   }

   public double getTotalMass()
   {
      if (!isTotalMassUpToDate)
      {
         totalMass = 0.0;
         for (InverseDynamicsJoint joint : joints)
            totalMass += joint.getSuccessor().getInertia().getMass();
         isTotalMassUpToDate = true;
      }
      return totalMass;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      if (!isCenterOfMassVelocityUpToDate)
      {
         centerOfMassVelocity.setAndScale(1.0 / getTotalMass(), getMomentum().getLinearPart());
         isCenterOfMassVelocityUpToDate = true;
      }

      return centerOfMassVelocity;
   }

   public FrameVector3DReadOnly getCenterOfMassAcceleration()
   {
      if (!isCenterOfMassAccelerationUpToDate)
      {
         centerOfMassAcceleration.setAndScale(1.0 / getTotalMass(), getMomentumRate().getLinearPart());
         isCenterOfMassAccelerationUpToDate = true;
      }

      return centerOfMassAcceleration;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return matrixFrame;
   }

   private static class RecursionStep
   {
      /**
       * The rigid-body for which this recursion is.
       */
      private final RigidBody rigidBody;
      /**
       * The reference frame in which the internal including the jacobian matrix is to be expressed
       * in.
       */
      private final ReferenceFrame matrixFrame;

      /**
       * Intermediate variable to store the unit-twist of the parent joint.
       */
      private final Twist jointUnitTwist;
      private final Twist intermediateTwist;
      /**
       * Coriolis acceleration.
       */
      private final SpatialAccelerationVector biasAcceleration;
      private final Momentum unitMomentum;
      private final Momentum intermediateMomentum;
      private final Wrench biasWrench;
      private final DenseMatrix64F centroidalMomentumMatrixBlock;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<RecursionStep> children = new ArrayList<>();
      private final RecursionStep parent;

      public RecursionStep(RigidBody rigidBody, RecursionStep parent, ReferenceFrame matrixFrame)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.matrixFrame = matrixFrame;

         if (isRoot())
         {
            jointUnitTwist = null;
            intermediateTwist = null;
            biasAcceleration = new SpatialAccelerationVector(getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), getBodyFixedFrame());
            unitMomentum = null;
            intermediateMomentum = null;
            biasWrench = null;
            centroidalMomentumMatrixBlock = null;
         }
         else
         {
            jointUnitTwist = new Twist();
            intermediateTwist = new Twist();
            biasAcceleration = new SpatialAccelerationVector();
            unitMomentum = new Momentum(matrixFrame);
            intermediateMomentum = new Momentum();
            biasWrench = new Wrench();
            centroidalMomentumMatrixBlock = new DenseMatrix64F(6, getJoint().getDegreesOfFreedom());
         }
      }

      public void passOne()
      {
         if (isRoot())
            return;

         RigidBodyInertia inertia = rigidBody.getInertia();
         ReferenceFrame inertiaFrame = inertia.getExpressedInFrame();

         biasAcceleration.set(parent.biasAcceleration);
         getJoint().getPredecessorTwist(intermediateTwist);
         biasAcceleration.changeFrame(getBodyFixedFrame(), intermediateTwist, parent.getBodyFixedFrame().getTwistOfFrame());
         biasAcceleration.changeBodyFrameNoRelativeAcceleration(getBodyFixedFrame());

         biasWrench.setToZero(inertiaFrame, inertiaFrame);
         inertia.computeDynamicWrenchInBodyCoordinates(biasAcceleration, getBodyFixedFrame().getTwistOfFrame(), biasWrench);
         biasWrench.changeFrame(matrixFrame);
      }

      public void passTwo()
      {
         if (isRoot())
            return;

         for (int i = 0; i < getJoint().getDegreesOfFreedom(); i++)
         {
            unitMomentum.setToZero();
            getJoint().getUnitTwist(i, jointUnitTwist);
            jointUnitTwist.changeFrame(matrixFrame);
            addToUnitMomentumRecursively(jointUnitTwist, unitMomentum);
            unitMomentum.getMatrixColumn(centroidalMomentumMatrixBlock, i);
         }
      }

      private void addToUnitMomentumRecursively(Twist ancestorUnitTwist, Momentum unitMomentumToAddTo)
      {
         RigidBodyInertia inertia = rigidBody.getInertia();

         ReferenceFrame inertiaFrame = inertia.getExpressedInFrame();

         intermediateTwist.set(ancestorUnitTwist);
         intermediateTwist.changeFrame(inertiaFrame);

         intermediateMomentum.setToZero(inertiaFrame);
         intermediateMomentum.compute(inertia, intermediateTwist);
         intermediateMomentum.changeFrame(matrixFrame);

         unitMomentumToAddTo.add(intermediateMomentum);

         for (int i = 0; i < children.size(); i++)
            children.get(i).addToUnitMomentumRecursively(ancestorUnitTwist, unitMomentumToAddTo);
      }

      public boolean isRoot()
      {
         return rigidBody.isRootBody();
      }

      public InverseDynamicsJoint getJoint()
      {
         return rigidBody.getParentJoint();
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }
   }
}