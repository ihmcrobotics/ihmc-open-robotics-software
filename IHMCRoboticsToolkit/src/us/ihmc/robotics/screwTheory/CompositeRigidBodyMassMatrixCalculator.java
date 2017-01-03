package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CompositeRigidBodyMassMatrixCalculator implements MassMatrixCalculator
{
   private final RigidBody[] allRigidBodiesInOrder;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final CompositeRigidBodyInertia[] crbInertiasInOrder;
   private final int[] parentMap;
   private final int[] massMatrixIndices;
   private final DenseMatrix64F massMatrix;
   private final Momentum[] unitMomenta;
   private final Twist tempTwist = new Twist();
   private int nMomentaInUse = 0;

   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody, ArrayList<InverseDynamicsJoint> jointsToIgnore)
   {
      allRigidBodiesInOrder = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      jointsInOrder = computeJointsInOrder(rootBody, jointsToIgnore.toArray(new InverseDynamicsJoint[0]));

      crbInertiasInOrder = createCompositeRigidBodyInertiasInOrder(allRigidBodiesInOrder.length);
      parentMap = ScrewTools.createParentMap(allRigidBodiesInOrder);
      massMatrixIndices = createMassMatrixIndices(allRigidBodiesInOrder);

      int degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrix = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);

      unitMomenta = createMomenta();
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
      for (int i = 0; i < allRigidBodiesInOrder.length; i++)
      {
         crbInertiasInOrder[i].set(allRigidBodiesInOrder[i].getInertia());
      }

      for (int bodyIndex = allRigidBodiesInOrder.length - 1; bodyIndex >= 0; bodyIndex--)
      {
         RigidBody currentBody = allRigidBodiesInOrder[bodyIndex];
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         CompositeRigidBodyInertia currentBodyInertia = crbInertiasInOrder[bodyIndex];
         GeometricJacobian motionSubspace = parentJoint.getMotionSubspace();

         setUnitMomenta(currentBodyInertia, motionSubspace);
         setDiagonalTerm(bodyIndex, motionSubspace);
         setOffDiagonalTerms(bodyIndex);
         buildCompositeRigidBodyInertia(bodyIndex);
      }
   }

   private void setUnitMomenta(CompositeRigidBodyInertia currentBodyInertia, GeometricJacobian motionSubspace)
   {
      nMomentaInUse = motionSubspace.getNumberOfColumns();

      for (int i = 0; i < nMomentaInUse; i++)
      {
         tempTwist.set(motionSubspace.getAllUnitTwists().get(i));
         tempTwist.changeFrame(currentBodyInertia.getExpressedInFrame());
         unitMomenta[i].compute(currentBodyInertia, tempTwist);
      }
   }

   private void setDiagonalTerm(int currentBodyIndex, GeometricJacobian motionSubspace)
   {
      setMassMatrixPart(currentBodyIndex, currentBodyIndex, motionSubspace);
   }
   
   private void setOffDiagonalTerms(int currentBodyIndex)
   {
      int parentIndex;
      int bodyIndex = currentBodyIndex;
      while (isValidParentIndex(parentIndex = parentMap[bodyIndex]))
      {
         ReferenceFrame parentFrame = allRigidBodiesInOrder[parentIndex].getInertia().getExpressedInFrame();
         changeFrameOfMomenta(parentFrame);
         bodyIndex = parentIndex;
         GeometricJacobian motionSubspace = allRigidBodiesInOrder[bodyIndex].getParentJoint().getMotionSubspace();
         setMassMatrixPart(currentBodyIndex, bodyIndex, motionSubspace);
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

   private void setMassMatrixPart(int rowBodyIndex, int colBodyIndex, GeometricJacobian motionSubspace)
   {
      int rowStart = massMatrixIndices[rowBodyIndex];
      int colStart = massMatrixIndices[colBodyIndex];

      for (int momentaIndex = 0; momentaIndex < nMomentaInUse; momentaIndex++)
      {
         Momentum unitMomentum = unitMomenta[momentaIndex];
         int massMatrixRow = rowStart + momentaIndex;

         for (int subspaceIndex = 0; subspaceIndex < motionSubspace.getNumberOfColumns(); subspaceIndex++)
         {
            Twist unitRelativeTwist = motionSubspace.getAllUnitTwists().get(subspaceIndex);
            unitRelativeTwist.changeFrame(unitMomentum.getExpressedInFrame());
            double entry = unitMomentum.computeKineticCoEnergy(unitRelativeTwist);
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

   private static InverseDynamicsJoint[] computeJointsInOrder(RigidBody rootBody, InverseDynamicsJoint... jointsToIgnore)
   {
      InverseDynamicsJoint[] jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      List<InverseDynamicsJoint> joints = new ArrayList<InverseDynamicsJoint>();
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
