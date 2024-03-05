package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

import java.util.*;

public class RigidBodyTwistCalculator implements RigidBodyTwistProvider
{

   private final MultiBodySystemReadOnly input;

   private final RigidBodyTwistHolder root;
   private final Map<RigidBodyReadOnly, RigidBodyTwistHolder> rigidBodyTwistHolders = new HashMap<>();

   private JointVelocityAccessor jointVelocityAccessor;
   private RigidBodyTwistProvider provider;

   public RigidBodyTwistCalculator(MultiBodySystemReadOnly input)
   {
      // TODO Woohoo! This is a stub!
      this.input = input;
      root = new RigidBodyTwistHolder(null, input.getRootBody());
      rigidBodyTwistHolders.put(input.getRootBody(), root);
      buildTree(root, input.getJointsToIgnore());

      provider = RigidBodyTwistProvider.toRigidBodyTwistProvider(this::getTwistOfBody, getInertialFrame());
   }

   private void buildTree(RigidBodyTwistHolder parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<? extends JointReadOnly> childrenJoints = parent.body.getChildrenJoints();

      for (JointReadOnly childJoint : childrenJoints)
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         RigidBodyReadOnly childBody = childJoint.getSuccessor();

         if (childBody == null)
            continue;

         RigidBodyTwistHolder childHolder = new RigidBodyTwistHolder(parent, childBody);
         parent.children.add(childHolder);
         rigidBodyTwistHolders.put(childBody, childHolder);
         buildTree(childHolder, jointsToIgnore);
      }
   }

   public void reset()
   {
      root.reset();
   }

   public void useJointVelocityState()
   {
      setJointVelocityAccessor((joint, matrixToPack) -> joint.getJointVelocity(0, matrixToPack));
   }

   public void useAllJointVelocityMatrix(DMatrix jointVelocities)
   {
      setJointVelocityAccessor((joint, matrixToPack) ->
                               {
                                  int[] jointDoFIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(joint);
                                  for (int i = 0; i < jointDoFIndices.length; i++)
                                  {
                                     matrixToPack.set(i, 0, jointVelocities.get(jointDoFIndices[i], 0));
                                  }
                               });
   }

   public void setJointVelocityAccessor(JointVelocityAccessor jointVelocityAccessor)
   {
      root.reset();
      this.jointVelocityAccessor = jointVelocityAccessor;
   }

   @Override
   public TwistReadOnly getTwistOfBody(RigidBodyReadOnly body)
   {
      RigidBodyTwistHolder holder = rigidBodyTwistHolders.get(body);
      if (holder == null)
         return null;
      return holder.getTwist();
   }

   @Override
   public TwistReadOnly getRelativeTwist(RigidBodyReadOnly base, RigidBodyReadOnly body)
   {
      return provider.getRelativeTwist(base, body);
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
   {
      return provider.getLinearVelocityOfBodyFixedPoint(base, body, bodyFixedPoint);
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return input.getInertialFrame();
   }

   private class RigidBodyTwistHolder
   {
      private final RigidBodyReadOnly body;
      private final Twist twist = new Twist();

      private final RigidBodyTwistHolder parent;
      private final List<RigidBodyTwistHolder> children = new ArrayList<>();

      private boolean dirty = true;
      private final DMatrixRMaj jointTwistMatrix;
      private final DMatrixRMaj jointVelocityMatrix;
      private final DMatrixRMaj jointMotionSubspaceMatrix;

      public RigidBodyTwistHolder(RigidBodyTwistHolder parent, RigidBodyReadOnly body)
      {
         this.parent = parent;
         this.body = body;

         if (parent == null)
         {
            jointTwistMatrix = null;
            jointVelocityMatrix = null;
            jointMotionSubspaceMatrix = null;
            twist.setToZero(body.getBodyFixedFrame(), getInertialFrame(), body.getBodyFixedFrame());
         }
         else
         {
            jointTwistMatrix = new DMatrixRMaj(Twist.SIZE, 1);
            int nDoFs = body.getParentJoint().getDegreesOfFreedom();
            jointVelocityMatrix = new DMatrixRMaj(nDoFs, 1);
            jointMotionSubspaceMatrix = new DMatrixRMaj(Twist.SIZE, nDoFs);
         }
      }

      public void reset()
      {
         dirty = true;
         for (int i = 0; i < children.size(); i++)
         {
            children.get(i).reset();
         }
      }

      public TwistReadOnly getTwist()
      {
         if (jointVelocityAccessor == null)
            throw new IllegalStateException("Joint velocity accessor has not been set.");

         if (dirty)
         {
            if (parent == null)
            {
               twist.setToZero(body.getBodyFixedFrame(), getInertialFrame(), body.getBodyFixedFrame());
            }
            else
            {
               JointReadOnly joint = body.getParentJoint();
               twist.setIncludingFrame(parent.getTwist());
               twist.changeFrame(joint.getFrameAfterJoint());

               jointVelocityAccessor.getJointVelocity(joint, jointVelocityMatrix);
               joint.getMotionSubspace(jointMotionSubspaceMatrix);
               CommonOps_DDRM.mult(jointMotionSubspaceMatrix, jointVelocityMatrix, jointTwistMatrix);
               twist.add(jointTwistMatrix);

               twist.changeFrame(body.getBodyFixedFrame());
               twist.setBodyFrame(body.getBodyFixedFrame());
            }
            dirty = false;
         }
         return twist;
      }
   }

   public interface JointVelocityAccessor
   {
      void getJointVelocity(JointReadOnly joint, DMatrix velocityToPack);
   }
}
