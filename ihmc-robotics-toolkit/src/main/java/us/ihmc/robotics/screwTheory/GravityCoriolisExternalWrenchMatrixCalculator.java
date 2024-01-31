package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Implements the same algorithm as {@link InverseDynamicsCalculator} but ignores joint
 * accelerations.
 * <p>
 * The resulting joint torques are thus a result of the centrifugal, Coriolis, and external wrenches
 * only.
 * </p>
 */
public class GravityCoriolisExternalWrenchMatrixCalculator
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;

   /** The root of the internal recursive algorithm. */
   private final RecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, RecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();

   /**
    * The output of this algorithm: the effort matrix for all the joints to consider resulting from
    * Coriolis, centrifugal, and external wrenches.
    */
   private final DMatrixRMaj jointTauMatrix;

   /** Whether the effort resulting from the Coriolis and centrifugal forces should be considered. */
   private final boolean considerCoriolisAndCentrifugalForces;
   /**
    * Extension of this algorithm into an acceleration provider that be used instead of a
    * {@link SpatialAccelerationCalculator}.
    */
   private final RigidBodyAccelerationProvider coriolisAccelerationProvider;

   /**
    * Creates a calculator for computing the joint efforts for all the descendants of the given
    * {@code rootBody}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    * 
    * @param rootBody the supporting body of the subtree to be evaluated by this calculator. Not
    *                 modified.
    */
   public GravityCoriolisExternalWrenchMatrixCalculator(RigidBodyReadOnly rootBody)
   {
      this(rootBody, true);
   }

   /**
    * Creates a calculator for computing the joint efforts for all the descendants of the given
    * {@code rootBody}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    * 
    * @param rootBody                             the supporting body of the subtree to be evaluated by
    *                                             this calculator. Not modified.
    * @param considerCoriolisAndCentrifugalForces whether the effort resulting from the Coriolis and
    *                                             centrifugal forces should be considered.
    */
   public GravityCoriolisExternalWrenchMatrixCalculator(RigidBodyReadOnly rootBody, boolean considerCoriolisAndCentrifugalForces)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), considerCoriolisAndCentrifugalForces);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    * 
    * @param input the definition of the system to be evaluated by this calculator.
    */
   public GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemReadOnly input)
   {
      this(input, true, true);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    * 
    * @param input                          the definition of the system to be evaluated by this
    *                                       calculator.
    * @param considerIgnoredSubtreesInertia whether the inertia of the ignored part(s) of the given
    *                                       multi-body system should be considered. When {@code true},
    *                                       this provides more accurate joint torques as they
    *                                       compensate for instance for the gravity acting on the
    *                                       ignored rigid-bodies, i.e. bodies which have an ancestor
    *                                       joint that is ignored as specified in the given
    *                                       {@code input}. When {@code false}, the resulting joint
    *                                       torques may be less accurate and this calculator may gain
    *                                       slight performance improvement.
    */
   public GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemReadOnly input, boolean considerIgnoredSubtreesInertia)
   {
      this(input, true, considerIgnoredSubtreesInertia);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    * 
    * @param input                                the definition of the system to be evaluated by this
    *                                             calculator.
    * @param considerCoriolisAndCentrifugalForces whether the effort resulting from the Coriolis and
    *                                             centrifugal forces should be considered.
    * @param considerIgnoredSubtreesInertia       whether the inertia of the ignored part(s) of the
    *                                             given multi-body system should be considered. When
    *                                             {@code true}, this provides more accurate joint
    *                                             torques as they compensate for instance for the
    *                                             gravity acting on the ignored rigid-bodies, i.e.
    *                                             bodies which have an ancestor joint that is ignored
    *                                             as specified in the given {@code input}. When
    *                                             {@code false}, the resulting joint torques may be
    *                                             less accurate and this calculator may gain slight
    *                                             performance improvement.
    */
   public GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemReadOnly input, boolean considerCoriolisAndCentrifugalForces,
                                                        boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;
      this.considerCoriolisAndCentrifugalForces = considerCoriolisAndCentrifugalForces;

      RigidBodyReadOnly rootBody = input.getRootBody();
      initialRecursionStep = new RecursionStep(rootBody, null, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());

      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.includeIgnoredSubtreeInertia();

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointTauMatrix = new DMatrixRMaj(nDoFs, 1);

      coriolisAccelerationProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(body -> rigidBodyToRecursionStepMap.get(body).rigidBodyAcceleration,
                                                                                                   input.getInertialFrame(),
                                                                                                   considerCoriolisAndCentrifugalForces,
                                                                                                   false);
   }

   private void buildMultiBodyTree(RecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      for (JointReadOnly childJoint : parent.rigidBody.getChildrenJoints())
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         RigidBodyReadOnly childBody = childJoint.getSuccessor();

         if (childBody != null)
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            RecursionStep child = new RecursionStep(childBody, parent, jointIndices);
            rigidBodyToRecursionStepMap.put(childBody, child);
            buildMultiBodyTree(child, jointsToIgnore);
         }
      }
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    * 
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(FrameTuple3DReadOnly gravity)
   {
      gravity.checkReferenceFrameMatch(input.getInertialFrame());
      setGravitionalAcceleration((Tuple3DReadOnly) gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    * 
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(Tuple3DReadOnly gravity)
   {
      SpatialAcceleration rootAcceleration = initialRecursionStep.rigidBodyAcceleration;
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().setAndNegate(gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    * 
    * @param gravity the gravitational linear acceleration along the z-axis, it is usually equal to
    *                {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravity)
   {
      setGravitionalAcceleration(0.0, 0.0, gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    * 
    * @param gravityX the gravitational linear acceleration along the x-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityY the gravitational linear acceleration along the y-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityZ the gravitational linear acceleration along the z-axis, it is usually equal to
    *                 {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravityX, double gravityY, double gravityZ)
   {
      SpatialAcceleration rootAcceleration = initialRecursionStep.rigidBodyAcceleration;
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().set(gravityX, gravityY, gravityZ);
      rootAcceleration.negate();
   }

   /**
    * Changes the spatial acceleration of the root. Even though the root is assumed to be non-moving,
    * the {@code rootAcceleration} is usually set to the opposite of the gravitational acceleration,
    * such that the effect of the gravity is naturally propagated to the entire system.
    * 
    * @param newRootAcceleration the new spatial acceleration of the root.
    * @throws ReferenceFrameMismatchException if any of the reference frames of
    *                                         {@code newRootAcceleration} does not match this
    *                                         calculator's root spatial acceleration's frames.
    */
   public void setRootAcceleration(SpatialAccelerationReadOnly newRootAcceleration)
   {
      initialRecursionStep.rigidBodyAcceleration.set(newRootAcceleration);
   }

   /**
    * Resets all the external wrenches that were added to the rigid-bodies.
    */
   public void setExternalWrenchesToZero()
   {
      initialRecursionStep.setExternalWrenchToZeroRecursive();
   }

   /**
    * Gets the internal reference to the external wrench associated with the given rigidBody.
    * <p>
    * Modify the return wrench to configure the wrench to be applied on this rigid-body.
    * </p>
    * 
    * @param rigidBody the query. Not modified.
    * @return the wrench associated to the query.
    */
   public FixedFrameWrenchBasics getExternalWrench(RigidBodyReadOnly rigidBody)
   {
      return rigidBodyToRecursionStepMap.get(rigidBody).externalWrench;
   }

   /**
    * Sets external wrench to apply to the given {@code rigidBody}.
    * 
    * @param rigidBody      the rigid-body to which the wrench is to applied. Not modified.
    * @param externalWrench the external wrench to apply to the rigid-body.
    */
   public void setExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
   {
      getExternalWrench(rigidBody).setMatchingFrame(externalWrench);
   }

   public void compute()
   {
      initialRecursionStep.passOne();
      initialRecursionStep.passTwo();
   }

   /**
    * Gets the definition of the multi-body system that was used to create this calculator.
    * 
    * @return this calculator input.
    */
   public MultiBodySystemReadOnly getInput()
   {
      return input;
   }

   /**
    * Gets the computed joint efforts.
    * 
    * @return this calculator output: the joint efforts.
    */
   public DMatrixRMaj getJointTauMatrix()
   {
      return jointTauMatrix;
   }

   /**
    * Gets the computed wrench for the given {@code joint}.
    * 
    * @param joint the query. Not modified.
    * @return the joint wrench or {@code null} if this calculator does not consider the given joint.
    */
   public WrenchReadOnly getComputedJointWrench(JointReadOnly joint)
   {
      RecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());
      if (recursionStep == null)
         return null;
      else
         return recursionStep.jointWrench;
   }

   /**
    * Gets the computed N-by-1 effort vector for the given {@code joint}, where N is the number of
    * degrees of freedom the joint has.
    * 
    * @param joint the query. Not modify.
    * @return the tau matrix.
    */
   public DMatrixRMaj getComputedJointTau(JointReadOnly joint)
   {
      RecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return null;
      else
         return recursionStep.tau;
   }

   /**
    * Writes the computed joint efforts into the given {@code joints}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    * 
    * @param joints the array of joints to write the effort into. Modified.
    */
   public void writeComputedJointWrenches(JointBasics[] joints)
   {
      for (JointBasics joint : joints)
         writeComputedJointWrench(joint);
   }

   /**
    * Writes the computed joint efforts into the given {@code joints}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    * 
    * @param joints the list of joints to write the effort into. Modified.
    */
   public void writeComputedJointWrenches(List<? extends JointBasics> joints)
   {
      for (int i = 0; i < joints.size(); i++)
         writeComputedJointWrench(joints.get(i));
   }

   /**
    * Writes the computed effort into the given {@code joint}.
    * <p>
    * If this calculator does not consider this joint, it remains unchanged.
    * </p>
    * 
    * @param joint the joint to retrieve the acceleration of and to store it. Modified.
    * @return {@code true} if the joint effort was modified, {@code false} otherwise.
    */
   public boolean writeComputedJointWrench(JointBasics joint)
   {
      WrenchReadOnly jointWrench = getComputedJointWrench(joint);

      if (jointWrench == null)
         return false;

      joint.setJointWrench(jointWrench);
      return true;
   }

   /**
    * Gets the rigid-body acceleration provider that uses accelerations computed in this calculator.
    * 
    * @return the acceleration provider backed by this calculator.
    */
   public RigidBodyAccelerationProvider getAccelerationProvider()
   {
      return coriolisAccelerationProvider;
   }

   /** Intermediate result used for garbage free operations. */
   private final SpatialForce jointForceFromChild = new SpatialForce();

   /**
    * Represents a single recursion step with all the intermediate variables needed.
    * 
    * @author Sylvain Bertrand
    */
   private final class RecursionStep
   {
      /**
       * The rigid-body for which this recursion is.
       */
      private final RigidBodyReadOnly rigidBody;
      /**
       * Body inertia: usually equal to {@code rigidBody.getInertial()}. However, if at least one child of
       * {@code rigidBody} is ignored, it is equal to this rigid-body inertia and the subtree inertia
       * attached to the ignored joint.
       */
      private final SpatialInertia bodyInertia;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private final RecursionStep parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<RecursionStep> children = new ArrayList<>();
      /**
       * Calculated joint wrench, before projection onto the joint motion subspace.
       */
      private final Wrench jointWrench;
      /**
       * User input: external wrench to be applied to this body.
       */
      private final FixedFrameWrenchBasics externalWrench;

      /**
       * The rigid-body spatial acceleration.
       */
      private final SpatialAcceleration rigidBodyAcceleration;
      /**
       * Intermediate variable for storing this joint acceleration.
       */
      private final SpatialAcceleration localJointAcceleration = new SpatialAcceleration();
      /**
       * Intermediate variable for storing this joint twist.
       */
      private final Twist localJointTwist = new Twist();
      /**
       * <tt>S</tt> is the 6-by-N matrix representing the motion subspace of the parent joint, where N is
       * the number of DoFs of the joint.
       */
      private final DMatrixRMaj S;
      /**
       * Computed joint effort.
       */
      private final DMatrixRMaj tau;
      /**
       * Computed joint wrench, before projection onto the joint motion subspace.
       */
      private final DMatrixRMaj jointWrenchMatrix;
      /**
       * Joint indices for storing {@code tau} in the main matrix {@code jointTauMatrix}.
       */
      private int[] jointIndices;

      public RecursionStep(RigidBodyReadOnly rigidBody, RecursionStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;
         rigidBodyAcceleration = new SpatialAcceleration(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());

         if (isRoot())
         {
            bodyInertia = null;
            jointWrench = null;
            externalWrench = null;
            S = null;
            tau = null;
            jointWrenchMatrix = null;
         }
         else
         {
            parent.children.add(this);
            int nDoFs = getJoint().getDegreesOfFreedom();

            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            jointWrench = new Wrench();
            externalWrench = new Wrench(getBodyFixedFrame(), getBodyFixedFrame());
            S = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, nDoFs);
            tau = new DMatrixRMaj(nDoFs, 1);
            jointWrenchMatrix = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            getJoint().getMotionSubspace(S);
         }
      }

      public void includeIgnoredSubtreeInertia()
      {
         if (!isRoot() && children.size() != rigidBody.getChildrenJoints().size())
         {
            for (JointReadOnly childJoint : rigidBody.getChildrenJoints())
            {
               if (input.getJointsToIgnore().contains(childJoint))
               {
                  SpatialInertia subtreeIneria = MultiBodySystemTools.computeSubtreeInertia(childJoint);
                  subtreeIneria.changeFrame(getBodyFixedFrame());
                  bodyInertia.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).includeIgnoredSubtreeInertia();
      }

      /**
       * First pass going from the root to the leaves.
       * <p>
       * Here the rigid-body accelerations are updated and the net wrenches resulting from the rigid-body
       * acceleration and velocity are computed.
       * </p>
       */
      public void passOne()
      {
         if (!isRoot())
         {
            if (getJoint().isMotionSubspaceVariable())
               getJoint().getMotionSubspace(S);

            rigidBodyAcceleration.setIncludingFrame(parent.rigidBodyAcceleration);

            TwistReadOnly bodyTwistToUse;

            if (considerCoriolisAndCentrifugalForces)
            {
               getJoint().getPredecessorTwist(localJointTwist);
               rigidBodyAcceleration.changeFrame(getBodyFixedFrame(), localJointTwist, parent.getBodyFixedFrame().getTwistOfFrame());
               bodyTwistToUse = getBodyTwist();

               if (getJoint().isMotionSubspaceVariable())
               {
                  SpatialAccelerationReadOnly jointBiasAcceleration = getJoint().getJointBiasAcceleration();
                  localJointAcceleration.setIncludingFrame(jointBiasAcceleration);
                  localJointAcceleration.changeFrame(getBodyFixedFrame());
                  localJointAcceleration.setBodyFrame(getBodyFixedFrame());
                  localJointAcceleration.setBaseFrame(parent.getBodyFixedFrame());
                  rigidBodyAcceleration.add(localJointAcceleration);
               }
            }
            else
            {
               rigidBodyAcceleration.changeFrame(getBodyFixedFrame());
               bodyTwistToUse = null;
            }

            rigidBodyAcceleration.setBodyFrame(getBodyFixedFrame());

            bodyInertia.computeDynamicWrench(rigidBodyAcceleration, bodyTwistToUse, jointWrench);
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).passOne();
         }
      }

      /**
       * Second pass going from leaves to the root.
       * <p>
       * The net wrenches are propagated upstream and summed up at each rigid-body to compute the joint
       * effort.
       * </p>
       */
      public void passTwo()
      {
         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).passTwo();
         }

         if (isRoot())
            return;

         jointWrench.sub(externalWrench);
         jointWrench.changeFrame(getFrameAfterJoint());

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            jointForceFromChild.setIncludingFrame(children.get(childIndex).jointWrench);
            jointForceFromChild.changeFrame(getFrameAfterJoint());
            jointWrench.add(jointForceFromChild);
         }

         jointWrench.get(jointWrenchMatrix);
         CommonOps_DDRM.multTransA(S, jointWrenchMatrix, tau);

         for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
         {
            jointTauMatrix.set(jointIndices[dofIndex], 0, tau.get(dofIndex, 0));
         }
      }

      /**
       * Resets the external wrenches from here down the leaves recursively.
       */
      public void setExternalWrenchToZeroRecursive()
      {
         if (!isRoot())
            externalWrench.setToZero();

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).setExternalWrenchToZeroRecursive();
         }
      }

      private MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }

      public MovingReferenceFrame getFrameAfterJoint()
      {
         return getJoint().getFrameAfterJoint();
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }

      public TwistReadOnly getBodyTwist()
      {
         return getBodyFixedFrame().getTwistOfFrame();
      }

      private boolean isRoot()
      {
         return parent == null;
      }
   }
}
