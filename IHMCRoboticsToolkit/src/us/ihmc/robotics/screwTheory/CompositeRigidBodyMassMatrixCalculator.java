package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CompositeRigidBodyMassMatrixCalculator implements MassMatrixCalculator
{
   private final RigidBody[] allBodiesExceptRoot;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final CompositeRigidBodyInertia[] crbInertiasInOrder;
   private final int[] parentMap;
   private final int[] massMatrixIndices;
   private final DenseMatrix64F massMatrix;
   private final Momentum[] unitMomenta;
   private final Twist tempTwist = new Twist();
   private int nMomentaInUse = 0;

   private final LinkedHashMap<InverseDynamicsJoint, int[]> srcJointIndices = new LinkedHashMap<>();

   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody, ArrayList<InverseDynamicsJoint> jointsToIgnore)
   {
      allBodiesExceptRoot = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      jointsInOrder = computeJointsInOrder(rootBody, jointsToIgnore);


      crbInertiasInOrder = createCompositeRigidBodyInertiasInOrder(allBodiesExceptRoot.length);
      parentMap = ScrewTools.createParentMap(allBodiesExceptRoot);
      massMatrixIndices = createMassMatrixIndices(allBodiesExceptRoot);

      int degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrix = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);

      unitMomenta = createMomenta();

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         srcJointIndices.put(joint, indices);
      }
   }


   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody)
   {
      this(rootBody, new ArrayList<InverseDynamicsJoint>());
   }

   @Override
   public void compute()
   {
      massMatrix.zero();

      // // TODO: 1/1/17  does this ever change?
      for (int i = 0; i < allBodiesExceptRoot.length; i++)
      {
         crbInertiasInOrder[i].set(allBodiesExceptRoot[i].getInertia());
      }

      for (int bodyIndex = allBodiesExceptRoot.length - 1; bodyIndex >= 0; bodyIndex--)
      {
         RigidBody currentBody = allBodiesExceptRoot[bodyIndex];
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         CompositeRigidBodyInertia currentBodyInertia = crbInertiasInOrder[bodyIndex];
         GeometricJacobian motionSubspace = parentJoint.getMotionSubspace();

         setUnitMomenta(currentBodyInertia, parentJoint);
         setDiagonalTerm(bodyIndex, parentJoint);
         setOffDiagonalTerms(bodyIndex);
         buildCompositeRigidBodyInertia(bodyIndex);
      }
   }

   private void setUnitMomenta(CompositeRigidBodyInertia currentBodyInertia, InverseDynamicsJoint joint)
   {
      nMomentaInUse = joint.getDegreesOfFreedom();

      for (int i = 0; i < nMomentaInUse; i++)
      {
         joint.getUnitTwist(i, tempTwist);
         tempTwist.changeFrame(currentBodyInertia.getExpressedInFrame());
         unitMomenta[i].compute(currentBodyInertia, tempTwist);
      }
   }

   private void setDiagonalTerm(int currentBodyIndex, InverseDynamicsJoint joint)
   {
      setMassMatrixPart(currentBodyIndex, currentBodyIndex, joint);
   }
   
   private void setOffDiagonalTerms(int currentBodyIndex)
   {
      int parentIndex;
      int bodyIndex = currentBodyIndex;
      while (isValidParentIndex(parentIndex = parentMap[bodyIndex]))
      {
         ReferenceFrame parentFrame = allBodiesExceptRoot[parentIndex].getInertia().getExpressedInFrame();
         changeFrameOfMomenta(parentFrame);
         bodyIndex = parentIndex;
         setMassMatrixPart(currentBodyIndex, bodyIndex, allBodiesExceptRoot[bodyIndex].getParentJoint());
      }
   }

   private void buildCompositeRigidBodyInertia(int currentBodyIndex)
   {
      CompositeRigidBodyInertia currentBodyInertia = crbInertiasInOrder[currentBodyIndex];
      if (isValidParentIndex(parentMap[currentBodyIndex]))
      {
         CompositeRigidBodyInertia parentBodyInertia = crbInertiasInOrder[parentMap[currentBodyIndex]];
         ReferenceFrame parentFrame = parentBodyInertia.getExpressedInFrame();
         currentBodyInertia.changeFrame(parentFrame);
         parentBodyInertia.add(currentBodyInertia);
      }
   }

   @Override
   public DenseMatrix64F getMassMatrix()
   {
      return massMatrix;
   }

   @Override
   public void getMassMatrix(DenseMatrix64F massMatrixToPack)
   {
      massMatrixToPack.set(massMatrix);
   }


   @Override
   public InverseDynamicsJoint[] getJointsInOrder()
   {
      return jointsInOrder;
   }

   private void changeFrameOfMomenta(ReferenceFrame desiredFrame)
   {
      for (int n = 0; n < nMomentaInUse; n++)
      {
         unitMomenta[n].changeFrame(desiredFrame);
      }
   }

   private final Twist tempUnitTwist = new Twist();

   private void setMassMatrixPart(int rowBodyIndex, int colBodyIndex, InverseDynamicsJoint joint)
   {
      int rowStart = massMatrixIndices[rowBodyIndex];
      int colStart = massMatrixIndices[colBodyIndex];

      for (int momentaIndex = 0; momentaIndex < nMomentaInUse; momentaIndex++)
      {
         Momentum unitMomentum = unitMomenta[momentaIndex];
         int massMatrixRow = rowStart + momentaIndex;

         for (int subspaceIndex = 0; subspaceIndex < joint.getDegreesOfFreedom(); subspaceIndex++)
         {
            joint.getUnitTwist(subspaceIndex, tempUnitTwist);
            tempUnitTwist.changeFrame(unitMomentum.getExpressedInFrame());
            double entry = unitMomentum.computeKineticCoEnergy(tempUnitTwist);
            int massMatrixCol = colStart + subspaceIndex;
            setMassMatrixSymmetrically(massMatrixRow, massMatrixCol, entry);
         }
      }
   }

   private void setMassMatrixSymmetrically(int row, int column, double entry)
   {
      massMatrix.set(row, column, entry);
      massMatrix.set(column, row, entry);
   }

   private static CompositeRigidBodyInertia[] createCompositeRigidBodyInertiasInOrder(int length)
   {
      CompositeRigidBodyInertia[] ret = new CompositeRigidBodyInertia[length];
      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = new CompositeRigidBodyInertia();
      }

      return ret;
   }

   private static int[] createMassMatrixIndices(RigidBody[] rigidBodiesInOrder)
   {
      int[] ret = new int[rigidBodiesInOrder.length];
      int currentIndex = 0;
      for (int i = 0; i < rigidBodiesInOrder.length; i++)
      {
         RigidBody rigidBody = rigidBodiesInOrder[i];
         ret[i] = currentIndex;
         currentIndex += rigidBody.getParentJoint().getDegreesOfFreedom();
      }

      return ret;
   }

   private static Momentum[] createMomenta()
   {
      Momentum[] ret = new Momentum[InverseDynamicsJoint.maxDoF];
      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = new Momentum();
      }

      return ret;
   }

   private static boolean isValidParentIndex(int parentIndex)
   {
      return parentIndex >= 0;
   }

   private static InverseDynamicsJoint[] computeJointsInOrder(RigidBody rootBody, ArrayList<InverseDynamicsJoint> jointsToIgnore)
   {
      InverseDynamicsJoint[] jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      ArrayList<InverseDynamicsJoint> joints = new ArrayList<>();
      joints.addAll(Arrays.asList(jointsInOrder));
      if (jointsToIgnore != null)
      {
         for (InverseDynamicsJoint joint : jointsToIgnore)
         {
            joints.remove(joint);
         }
      }

      return joints.toArray(new InverseDynamicsJoint[joints.size()]);
   }
}
