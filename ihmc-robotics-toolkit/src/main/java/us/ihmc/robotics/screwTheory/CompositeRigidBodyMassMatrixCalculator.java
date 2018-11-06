package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;

public class CompositeRigidBodyMassMatrixCalculator implements MassMatrixCalculator
{
   private final RigidBody[] allBodiesExceptRoot;
   private final JointBasics[] jointsInOrder;
   private final SpatialInertia[] crbInertiasInOrder;
   private final int[] parentMap;
   private final int[] massMatrixIndices;
   private final DenseMatrix64F massMatrix;
   private final Momentum[] unitMomenta;
   private final Twist tempTwist = new Twist();
   private int nMomentaInUse = 0;

   private final LinkedHashMap<JointBasics, int[]> srcJointIndices = new LinkedHashMap<>();

   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody, ArrayList<JointBasics> jointsToIgnore)
   {
      allBodiesExceptRoot = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      jointsInOrder = computeJointsInOrder(rootBody, jointsToIgnore);


      crbInertiasInOrder = createCompositeRigidBodyInertiasInOrder(allBodiesExceptRoot.length);
      parentMap = ScrewTools.createParentMap(allBodiesExceptRoot);
      massMatrixIndices = createMassMatrixIndices(allBodiesExceptRoot);

      int degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrix = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);

      unitMomenta = createMomenta();

      for (JointBasics joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         srcJointIndices.put(joint, indices);
      }
   }


   public CompositeRigidBodyMassMatrixCalculator(RigidBody rootBody)
   {
      this(rootBody, new ArrayList<JointBasics>());
   }

   @Override
   public void compute()
   {
      massMatrix.zero();

      // // TODO: 1/1/17  does this ever change?
      for (int i = 0; i < allBodiesExceptRoot.length; i++)
      {
         crbInertiasInOrder[i].setIncludingFrame(allBodiesExceptRoot[i].getInertia());
      }

      for (int bodyIndex = allBodiesExceptRoot.length - 1; bodyIndex >= 0; bodyIndex--)
      {
         RigidBody currentBody = allBodiesExceptRoot[bodyIndex];
         JointBasics parentJoint = currentBody.getParentJoint();
         SpatialInertia currentBodyInertia = crbInertiasInOrder[bodyIndex];

         setUnitMomenta(currentBodyInertia, parentJoint);
         setDiagonalTerm(bodyIndex, parentJoint);
         setOffDiagonalTerms(bodyIndex);
         buildCompositeRigidBodyInertia(bodyIndex);
      }
   }

   private void setUnitMomenta(SpatialInertia currentBodyInertia, JointBasics joint)
   {
      nMomentaInUse = joint.getDegreesOfFreedom();

      for (int i = 0; i < nMomentaInUse; i++)
      {
         tempTwist.setIncludingFrame(joint.getUnitTwists().get(i));
         tempTwist.changeFrame(currentBodyInertia.getReferenceFrame());
         unitMomenta[i].setReferenceFrame(currentBodyInertia.getReferenceFrame());
         unitMomenta[i].compute(currentBodyInertia, tempTwist);
      }
   }

   private void setDiagonalTerm(int currentBodyIndex, JointBasics joint)
   {
      setMassMatrixPart(currentBodyIndex, currentBodyIndex, joint);
   }
   
   private void setOffDiagonalTerms(int currentBodyIndex)
   {
      int parentIndex;
      int bodyIndex = currentBodyIndex;
      while (isValidParentIndex(parentIndex = parentMap[bodyIndex]))
      {
         ReferenceFrame parentFrame = allBodiesExceptRoot[parentIndex].getInertia().getReferenceFrame();
         changeFrameOfMomenta(parentFrame);
         bodyIndex = parentIndex;
         setMassMatrixPart(currentBodyIndex, bodyIndex, allBodiesExceptRoot[bodyIndex].getParentJoint());
      }
   }

   private void buildCompositeRigidBodyInertia(int currentBodyIndex)
   {
      SpatialInertia currentBodyInertia = crbInertiasInOrder[currentBodyIndex];
      if (isValidParentIndex(parentMap[currentBodyIndex]))
      {
         SpatialInertia parentBodyInertia = crbInertiasInOrder[parentMap[currentBodyIndex]];
         ReferenceFrame parentFrame = parentBodyInertia.getReferenceFrame();
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
   public JointBasics[] getJointsInOrder()
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

   private void setMassMatrixPart(int rowBodyIndex, int colBodyIndex, JointBasics joint)
   {
      int rowStart = massMatrixIndices[rowBodyIndex];
      int colStart = massMatrixIndices[colBodyIndex];

      for (int momentaIndex = 0; momentaIndex < nMomentaInUse; momentaIndex++)
      {
         Momentum unitMomentum = unitMomenta[momentaIndex];
         int massMatrixRow = rowStart + momentaIndex;

         for (int subspaceIndex = 0; subspaceIndex < joint.getDegreesOfFreedom(); subspaceIndex++)
         {
            tempUnitTwist.setIncludingFrame(joint.getUnitTwists().get(subspaceIndex));
            tempUnitTwist.changeFrame(unitMomentum.getReferenceFrame());
            double entry = unitMomentum.dot(tempUnitTwist);
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

   private static SpatialInertia[] createCompositeRigidBodyInertiasInOrder(int length)
   {
      SpatialInertia[] ret = new SpatialInertia[length];
      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = new SpatialInertia();
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
      Momentum[] ret = new Momentum[JointBasics.MAX_NUMBER_OF_DOFS];
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

   private static JointBasics[] computeJointsInOrder(RigidBody rootBody, ArrayList<JointBasics> jointsToIgnore)
   {
      JointBasics[] jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      ArrayList<JointBasics> joints = new ArrayList<>();
      joints.addAll(Arrays.asList(jointsInOrder));
      if (jointsToIgnore != null)
      {
         for (JointBasics joint : jointsToIgnore)
         {
            joints.remove(joint);
         }
      }

      return joints.toArray(new JointBasics[joints.size()]);
   }

   public SpatialInertia getBaseBodyCompositeInertia()
   {
      return crbInertiasInOrder[0];
   }
}
