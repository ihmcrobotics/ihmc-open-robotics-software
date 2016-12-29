package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class GravityCoriolisExternalWrenchMatrixCalculator
{
   private final InverseDynamicsJoint floatingJoint;
   private final ArrayList<InverseDynamicsJoint> jointsToIgnore;

   private final int degreesOfFreedom;
   private final int[] coriolisMatrixIndices;

   private final ArrayList<RigidBody> listOfBodiesWithExternalWrenches = new ArrayList<>();
   private final ArrayList<InverseDynamicsJoint> jointsToOptimizeFor = new ArrayList<>();

   private final RigidBody[] bodiesToOptimizeFor;

   private final LinkedHashMap<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final LinkedHashMap<RigidBody, Wrench> netWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final LinkedHashMap<InverseDynamicsJoint, Wrench> jointWrenches = new LinkedHashMap<InverseDynamicsJoint, Wrench>();
   private final TwistCalculator twistCalculator;

   private final SpatialAccelerationVector tempAcceleration = new SpatialAccelerationVector();
   private final Twist tempTwist = new Twist();

   private final DenseMatrix64F coriolisMatrix;

   private final boolean doVelocityTerms;

   public GravityCoriolisExternalWrenchMatrixCalculator(List<InverseDynamicsJoint> jointsToIgnore, InverseDynamicsJoint[] jointsToOptimizeFor,
        FloatingInverseDynamicsJoint floatingJoint, TwistCalculator twistCalculator, boolean doVelocityTerms)
   {
      this.floatingJoint = floatingJoint;
      this.jointsToIgnore = new ArrayList<>(jointsToIgnore);
      this.twistCalculator = twistCalculator;

      this.doVelocityTerms = doVelocityTerms;

      for (int i = 0; i < jointsToOptimizeFor.length; i++)
         this.jointsToOptimizeFor.add(jointsToOptimizeFor[i]);
      bodiesToOptimizeFor = ScrewTools.computeRigidBodiesAfterThisJoint(floatingJoint);

      populateMapsAndLists();

      degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      coriolisMatrixIndices = createMassMatrixIndices(this.jointsToOptimizeFor);
      coriolisMatrix = new DenseMatrix64F(degreesOfFreedom, 1);
   }

   private void populateMapsAndLists()
   {
      for (int i = 0; i < jointsToOptimizeFor.size(); i++)
         jointWrenches.put(jointsToOptimizeFor.get(i), new Wrench());

      for (int i = 0; i < bodiesToOptimizeFor.length; i++)
      {
         RigidBody currentBody = bodiesToOptimizeFor[i];
         ReferenceFrame bodyFixedFrame = currentBody.getBodyFixedFrame();

         netWrenches.put(currentBody, new Wrench(bodyFixedFrame, bodyFixedFrame));

         if (externalWrenches.get(currentBody) == null)
         {
            listOfBodiesWithExternalWrenches.add(currentBody);
            externalWrenches.put(currentBody, new Wrench(bodyFixedFrame, bodyFixedFrame));
         }
      }
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

   public void reset()
   {
      for (int i = 0; i < listOfBodiesWithExternalWrenches.size(); i++)
      {
         Wrench externalWrench = externalWrenches.get(listOfBodiesWithExternalWrenches.get(i));
         externalWrench.setToZero(externalWrench.getBodyFrame(), externalWrench.getExpressedInFrame());
      }
   }

   public void compute()
   {
      MatrixTools.setToZero(coriolisMatrix);

      computeNetWrenches();
      computeJointWrenchesAndTorques();
   }

   public void setExternalWrench(RigidBody rigidBody, Wrench externalWrench)
   {
      if (externalWrenches.containsKey(rigidBody))
         externalWrenches.get(rigidBody).set(externalWrench);
      else
         throw new RuntimeException("External Wrench map does not contain key " + rigidBody.getName() + ".");
   }

   private void computeNetWrenches()
   {
      for (int bodyIndex = 0; bodyIndex < bodiesToOptimizeFor.length; bodyIndex++)
      {
         RigidBody body = bodiesToOptimizeFor[bodyIndex];
         Wrench netWrench = netWrenches.get(body);
         twistCalculator.getTwistOfBody(tempTwist, body);
         if (!doVelocityTerms)
            tempTwist.setToZero();
         tempAcceleration.setToZero(body.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), body.getBodyFixedFrame());
         body.getInertia().computeDynamicWrenchInBodyCoordinates(netWrench, tempAcceleration, tempTwist);
      }
   }

   private final Wrench wrenchExertedByChild = new Wrench();
   private final DenseMatrix64F tempTauMatrix = new DenseMatrix64F(6);

   private void computeJointWrenchesAndTorques()
   {
      for (int jointIndex = jointsToOptimizeFor.size() - 1; jointIndex >= 0; jointIndex--)
      {
         InverseDynamicsJoint joint = jointsToOptimizeFor.get(jointIndex);

         RigidBody successor = joint.getSuccessor();

         Wrench jointWrench = jointWrenches.get(joint);
         jointWrench.set(netWrenches.get(successor));

         Wrench externalWrench = externalWrenches.get(successor);
         jointWrench.sub(externalWrench);

         List<InverseDynamicsJoint> childrenJoints = successor.getChildrenJoints();

         for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
         {
            InverseDynamicsJoint child = childrenJoints.get(childIndex);
            if (!jointsToIgnore.contains(child) && jointsToOptimizeFor.contains(child))
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

         int jointDoFs = joint.getDegreesOfFreedom();
         tempTauMatrix.reshape(jointDoFs, 1);
         joint.getTauMatrix(tempTauMatrix);

         int startIndex = coriolisMatrixIndices[jointIndex];
         for (int dof = 0; dof < jointDoFs; dof++)
            coriolisMatrix.set(startIndex + dof, 0, tempTauMatrix.get(dof));
      }
   }

   public int getNumberOfDegreesOfFreedom()
   {
      return degreesOfFreedom;
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
