package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CompositeRigidBodyMassMatrixCalculator implements MassMatrixCalculator
{
   private final RigidBody rootBody;
   private final RigidBody[] allRigidBodiesInOrder;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final ArrayList<InverseDynamicsJoint> jointsToIgnore;
   private final CompositeRigidBodyInertia[] crbInertiasInOrder;
   private final int[] parentMap;
   private final int[] massMatrixIndices;
   private final DenseMatrix64F massMatrix;
   private final Momentum[] unitMomenta;
   private final Twist tempTwist = new Twist();
   private int nMomentaInUse = 0;

   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody, ArrayList<InverseDynamicsJoint> jointsToIgnore)
   {
      this.rootBody = rootBody;
      this.jointsToIgnore = new ArrayList<InverseDynamicsJoint>(jointsToIgnore);
      
      ArrayList<RigidBody> rigidBodiesInOrderList = new ArrayList<RigidBody>();
      ArrayList<InverseDynamicsJoint> jointsInOrderList = new ArrayList<InverseDynamicsJoint>();
      populateMapsAndLists(jointsInOrderList, rigidBodiesInOrderList);
      allRigidBodiesInOrder = rigidBodiesInOrderList.toArray(new RigidBody[0]);
      jointsInOrder = jointsInOrderList.toArray(new InverseDynamicsJoint[0]);
      
      crbInertiasInOrder = createCrbInertiasInOrder(allRigidBodiesInOrder.length);
      parentMap = ScrewTools.createParentMap(allRigidBodiesInOrder);
      massMatrixIndices = createMassMatrixIndices(allRigidBodiesInOrder);
      int size = determineSize(allRigidBodiesInOrder);
      massMatrix = new DenseMatrix64F(size, size);
      unitMomenta = createMomenta();
   }

   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody)
   {
      this(rootBody, new ArrayList<InverseDynamicsJoint>());
   }
   
   @Override
   public void compute()
   {
      MatrixTools.setToZero(massMatrix);

      for (int i = 0; i < allRigidBodiesInOrder.length; i++)
      {
         crbInertiasInOrder[i].set(allRigidBodiesInOrder[i].getInertia());
      }

      for (int i = allRigidBodiesInOrder.length - 1; i >= 0; i--)
      {
         RigidBody currentBody = allRigidBodiesInOrder[i];
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         CompositeRigidBodyInertia currentBodyInertia = crbInertiasInOrder[i];
         
         setUnitMomenta(currentBodyInertia, parentJoint);
         setDiagonalTerm(i, parentJoint);
         setOffDiagonalTerms(i);
         buildCrbInertia(i);
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

   private void setDiagonalTerm(int i, InverseDynamicsJoint joint)
   {
      setMassMatrixPart(i, i, joint);
   }
   
   private void setOffDiagonalTerms(int i)
   {
      int parentIndex;
      int j = i;
      while (isValidParentIndex(parentIndex = parentMap[j]))
      {
         ReferenceFrame parentFrame = allRigidBodiesInOrder[parentIndex].getInertia().getExpressedInFrame();
         changeFrameOfMomenta(parentFrame);
         j = parentIndex;
         setMassMatrixPart(i, j, allRigidBodiesInOrder[j].getParentJoint());
      }
   }

   private void buildCrbInertia(int i)
   {
      CompositeRigidBodyInertia currentBodyInertia = crbInertiasInOrder[i];
      if (isValidParentIndex(parentMap[i]))
      {
         CompositeRigidBodyInertia parentBodyInertia = crbInertiasInOrder[parentMap[i]];
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

   private void setMassMatrixPart(int i, int j, InverseDynamicsJoint joint)
   {
      int rowStart = massMatrixIndices[i];
      int colStart = massMatrixIndices[j];

      for (int m = 0; m < nMomentaInUse; m++)
      {
         Momentum unitMomentum = unitMomenta[m];
         int massMatrixRow = rowStart + m;

         for (int n = 0; n < joint.getDegreesOfFreedom(); n++)
         {
            joint.getUnitTwist(n, tempUnitTwist);
            tempUnitTwist.changeFrame(unitMomentum.getExpressedInFrame());
            double entry = unitMomentum.computeKineticCoEnergy(tempUnitTwist);
            int massMatrixCol = colStart + n;
            setMassMatrixSymmetrically(massMatrixRow, massMatrixCol, entry);
         }
      }
   }

   private void setMassMatrixSymmetrically(int row, int column, double entry)
   {
      massMatrix.set(row, column, entry);
      massMatrix.set(column, row, entry);
   }

   private static CompositeRigidBodyInertia[] createCrbInertiasInOrder(int length)
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

   private static int determineSize(RigidBody[] rigidBodiesInOrder)
   {
      int ret = 0;
      for (RigidBody rigidBody : rigidBodiesInOrder)
      {
         ret += rigidBody.getParentJoint().getDegreesOfFreedom();
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

   // TODO: Use ScrewTools
   private void populateMapsAndLists(ArrayList<InverseDynamicsJoint> jointsInOrderListToPack, ArrayList<RigidBody> rigidBodiesInOrderListToPack)
   {
      ArrayList<RigidBody> morgue = new ArrayList<RigidBody>();
      
      morgue.add(rootBody);

      while (!morgue.isEmpty())
      {
         RigidBody currentBody = morgue.get(0);

         if (!currentBody.isRootBody())
         {
            rigidBodiesInOrderListToPack.add(currentBody);
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
                     if (rigidBodiesInOrderListToPack.contains(successor))
                     {
                        throw new RuntimeException("This algorithm doesn't do loops.");
                     }

                     jointsInOrderListToPack.add(joint);
                     morgue.add(successor);
                  }
               }
            }
         }

         morgue.remove(currentBody);
      }
   }
}
