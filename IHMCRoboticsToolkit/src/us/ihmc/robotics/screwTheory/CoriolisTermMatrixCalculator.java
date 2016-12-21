package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class CoriolisTermMatrixCalculator
{
   private final RigidBody rootBody;
   private final InverseDynamicsJoint floatingJoint;
   private final ArrayList<InverseDynamicsJoint> jointsToIgnore;

   private final int degreesOfFreedom;
   private final int[] coriolisMatrixIndices;

   private final ArrayList<RigidBody> allBodies = new ArrayList<>();
   private final ArrayList<RigidBody> allBodiesExceptRoot = new ArrayList<>();
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<>();
   private final LinkedHashMap<RigidBody, Wrench> netWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final LinkedHashMap<InverseDynamicsJoint, Wrench> jointWrenches = new LinkedHashMap<InverseDynamicsJoint, Wrench>();
   private final TwistCalculator twistCalculator;

   private final SpatialAccelerationVector tempAcceleration = new SpatialAccelerationVector();
   private final Twist tempTwist = new Twist();

   private final DenseMatrix64F coriolisMatrix;

   private final boolean doVelocityTerms;

   public CoriolisTermMatrixCalculator(List<InverseDynamicsJoint> jointsToIgnore, TwistCalculator twistCalculator, boolean doVelocityTerms)
   {
      this.rootBody = twistCalculator.getRootBody();
      this.floatingJoint = rootBody.getParentJoint();
      this.jointsToIgnore = new ArrayList<>(jointsToIgnore);
      this.twistCalculator = twistCalculator;

      this.doVelocityTerms = doVelocityTerms;

      populateMapsAndLists();

      degreesOfFreedom = determineSize(allJoints);
      coriolisMatrixIndices = createMassMatrixIndices(allJoints);
      coriolisMatrix = new DenseMatrix64F(degreesOfFreedom, 1);
   }

   private void populateMapsAndLists()
   {
      ArrayList<RigidBody> morgue = new ArrayList<>();
      morgue.add(rootBody);

      while (!morgue.isEmpty())
      {
         RigidBody currentBody = morgue.get(0);

         ReferenceFrame bodyFixedFrame = currentBody.getBodyFixedFrame();

         allBodies.add(currentBody);
         if (!currentBody.isRootBody())
         {
            allBodiesExceptRoot.add(currentBody);
            netWrenches.put(currentBody, new Wrench(bodyFixedFrame, bodyFixedFrame));
         }

         if (currentBody.hasChildrenJoints())
         {
            List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
            for (InverseDynamicsJoint joint : childrenJoints)
            {
               if (!jointsToIgnore.contains(joint))
               {
                  RigidBody successor = joint.getSuccessor();
                  if (successor != null)
                  {
                     if (allBodiesExceptRoot.contains(successor))
                     {
                        throw new RuntimeException("This algorithm doesn't do loops.");
                     }

                     allJoints.add(joint);
                     jointWrenches.put(joint, new Wrench());
                     morgue.add(successor);
                  }
               }
            }
         }

         morgue.remove(currentBody);
      }
   }

   private static int determineSize(List<InverseDynamicsJoint> jointsInOrder)
   {
      int ret = 0;
      for (int i = 0; i < jointsInOrder.size(); i++)
      {
         InverseDynamicsJoint joint = jointsInOrder.get(i);
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   private static int[] createMassMatrixIndices(List<InverseDynamicsJoint> jointsInOrder)
   {
      int[] ret = new int[jointsInOrder.size()];
      int currentIndex = 0;
      for (int i = 0; i < jointsInOrder.size(); i++)
      {
         InverseDynamicsJoint joint = jointsInOrder.get(i);
         ret[i] = currentIndex;
         currentIndex += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public void compute()
   {
      MatrixTools.setToZero(coriolisMatrix);

      computeNetWrenches();
      computeJointWrenchesAndTorques();
   }

   private void computeNetWrenches()
   {
      for (int bodyIndex = 0; bodyIndex < allBodiesExceptRoot.size(); bodyIndex++)
      {
         RigidBody body = allBodiesExceptRoot.get(bodyIndex);
         Wrench netWrench = netWrenches.get(body);
         twistCalculator.getTwistOfBody(tempTwist, body);
         if (!doVelocityTerms)
            tempTwist.setToZero();
         tempAcceleration.setToZero();
         body.getInertia().computeDynamicWrenchInBodyCoordinates(netWrench, tempAcceleration, tempTwist);
      }
   }

   private final Wrench wrenchExertedByChild = new Wrench();
   private final DenseMatrix64F tempTauMatrix = new DenseMatrix64F();

   private void computeJointWrenchesAndTorques()
   {
      for (int jointIndex = allJoints.size() - 1; jointIndex >= 0; jointIndex--)
      {
         InverseDynamicsJoint joint = allJoints.get(jointIndex);

         RigidBody successor = joint.getSuccessor();

         Wrench jointWrench = jointWrenches.get(joint);
         jointWrench.set(netWrenches.get(successor));

         List<InverseDynamicsJoint> childrenJoints = successor.getChildrenJoints();

         for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
         {
            InverseDynamicsJoint child = childrenJoints.get(childIndex);
            if (!jointsToIgnore.contains(child))
            {
               Wrench wrenchExertedOnChild = jointWrenches.get(child);
               ReferenceFrame successorFrame = successor.getBodyFixedFrame();

               wrenchExertedByChild.set(wrenchExertedOnChild);
               wrenchExertedByChild.changeBodyFrameAttachedToSameBody(successorFrame);
               wrenchExertedByChild.scale(-1.0); // Action = -reaction
               wrenchExertedByChild.changeFrame(jointWrench.getExpressedInFrame());
               jointWrench.sub(wrenchExertedByChild);
            }
         }

         joint.setTorqueFromWrench(jointWrench);

         int startIndex = coriolisMatrixIndices[jointIndex];
         joint.getTauMatrix(tempTauMatrix);
         for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
            coriolisMatrix.set(startIndex + dof, 0, tempTauMatrix.get(dof));
      }
   }

   public void getCoriolisMatrix(DenseMatrix64F coriolisMatrixToPack)
   {
      coriolisMatrixToPack.set(coriolisMatrix);
   }

   public void getFloatingBaseCoriolisMatrix(DenseMatrix64F floatingBaseCoriolisMatrixToPack)
   {
      CommonOps.extract(coriolisMatrix, 0, floatingJoint.getDegreesOfFreedom(), 0, 1, floatingBaseCoriolisMatrixToPack, 0, 0);
   }

   public void getBodyCoriolisMatrix(DenseMatrix64F bodyCoriolisMatrixToPack)
   {
      CommonOps.extract(coriolisMatrix, floatingJoint.getDegreesOfFreedom(), degreesOfFreedom - floatingJoint.getDegreesOfFreedom(), 0, 1, bodyCoriolisMatrixToPack, 0, 0);
   }
}
