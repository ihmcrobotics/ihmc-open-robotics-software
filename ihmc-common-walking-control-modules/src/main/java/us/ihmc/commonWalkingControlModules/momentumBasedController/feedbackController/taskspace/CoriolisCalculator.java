package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

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
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class CoriolisCalculator
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

   public CoriolisCalculator(RigidBodyReadOnly rootBody)
   {
      this(rootBody, true);
   }

   public CoriolisCalculator(RigidBodyReadOnly rootBody, boolean considerIgnoredSubtreesInertia)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), considerIgnoredSubtreesInertia);
   }

   public CoriolisCalculator(MultiBodySystemReadOnly input)
   {
      this(input, true);
   }

   public CoriolisCalculator(MultiBodySystemReadOnly input, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;

      RigidBodyReadOnly rootBody = input.getRootBody();
      initialRecursionStep = new RecursionStep(rootBody, null, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());

      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.includeIgnoredSubtreeInertia();

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointTauMatrix = new DMatrixRMaj(nDoFs, 1);
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

   private final DMatrixRMaj jointVelocitiesAlt = new DMatrixRMaj(20, 1);
   private final DMatrixRMaj coriolis = new DMatrixRMaj(20, 20);

   public void compute()
   {
      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointVelocitiesAlt.reshape(nDoFs, 1);
      coriolis.reshape(nDoFs, nDoFs);

      for (int i = 0; i < nDoFs; i++)
      {
         jointVelocitiesAlt.zero();
         jointVelocitiesAlt.set(i, 0, 1.0);

         initialRecursionStep.passOne();
         initialRecursionStep.passTwo();

         CommonOps_DDRM.insert(jointTauMatrix, coriolis, 0, i);
      }
   }

   public DMatrixRMaj getCoriolis()
   {
      return coriolis;
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
       * The rigid-body spatial acceleration.
       */
      private final SpatialAcceleration rigidBodyAcceleration;
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

      private final DMatrixRMaj twistMatrix = new DMatrixRMaj(6, 1);
      private final DMatrixRMaj velocityMatrix = new DMatrixRMaj(6, 1);

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
            rigidBodyAcceleration.setIncludingFrame(parent.rigidBodyAcceleration);

            velocityMatrix.reshape(getJoint().getDegreesOfFreedom(), 1);
            int[] indices = input.getJointMatrixIndexProvider().getJointDoFIndices(getJoint());
            for (int i = 0; i < indices.length; i++)
            {
               velocityMatrix.set(i, 0, jointVelocitiesAlt.get(indices[i], 0));
            }
            CommonOps_DDRM.mult(S, velocityMatrix, twistMatrix);
            localJointTwist.setIncludingFrame(getBodyFixedFrame(), parent.getBodyFixedFrame(), getBodyFixedFrame(), twistMatrix);
            localJointTwist.invert();
            localJointTwist.changeFrame(parent.getBodyFixedFrame());

            getJoint().getPredecessorTwist(localJointTwist);
            rigidBodyAcceleration.changeFrame(getBodyFixedFrame(), localJointTwist, parent.getBodyFixedFrame().getTwistOfFrame());
            TwistReadOnly bodyTwistToUse = getBodyTwist();

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
