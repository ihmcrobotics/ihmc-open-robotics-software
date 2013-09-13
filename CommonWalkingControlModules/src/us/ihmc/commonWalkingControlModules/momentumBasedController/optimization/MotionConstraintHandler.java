package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.apache.commons.lang.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.utilities.CheckTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.NullspaceCalculator;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.ConvectiveTermCalculator;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.PointJacobian;
import us.ihmc.utilities.screwTheory.PointJacobianConvectiveTermCalculator;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

/**
 * @author twan
 *         Date: 4/30/13
 */
public class MotionConstraintHandler
{
   private final InverseDynamicsJoint[] jointsInOrder;
   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();
   private final int nDegreesOfFreedom;

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   private final DenseMatrix64F taskSpaceAccelerationMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final List<DenseMatrix64F> jList = new ArrayList<DenseMatrix64F>();
   private final List<DenseMatrix64F> pList = new ArrayList<DenseMatrix64F>();
   private final List<MutableDouble> weightList = new ArrayList<MutableDouble>();

   private final DenseMatrix64F j = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F p = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F w = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F jBlockCompact = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F nCompactBlock = new DenseMatrix64F(1, 1);

   private final ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();
   private final PointJacobian pointJacobian = new PointJacobian();

   private int motionConstraintIndex = 0;
   private final FrameVector pPointVelocity = new FrameVector();
   private final PointJacobianConvectiveTermCalculator pointJacobianConvectiveTermCalculator;

   private MotionConstraintListener motionConstraintListener;
   
   public MotionConstraintHandler(InverseDynamicsJoint[] jointsInOrder, TwistCalculator twistCalculator)
   {
      this.jointsInOrder = jointsInOrder;
      this.nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         columnsForJoints.put(joint, ScrewTools.computeIndicesForJoint(jointsInOrder, joint));
      }

      pointJacobianConvectiveTermCalculator = new PointJacobianConvectiveTermCalculator(twistCalculator);
   }

   public void setMotionConstraintListener(MotionConstraintListener motionConstraintListener)
   {
      if (this.motionConstraintListener != null) throw new RuntimeException("Motion Constraint Listener was already set!");
      this.motionConstraintListener = motionConstraintListener;
   }
   
   public void reset()
   {
      motionConstraintIndex = 0;
   }

   public void setDesiredPointAcceleration(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase,
           DenseMatrix64F selectionMatrix, double weight)
   {
      pointJacobian.set(jacobian, bodyFixedPoint);
      pointJacobian.compute();
      desiredAccelerationWithRespectToBase.changeFrame(jacobian.getBaseFrame());

      DenseMatrix64F pointJacobianMatrix = pointJacobian.getJacobianMatrix();

      jBlockCompact.reshape(selectionMatrix.getNumRows(), pointJacobianMatrix.getNumCols());

      
      DenseMatrix64F jFullBlock = getMatrixFromList(jList, motionConstraintIndex, jBlockCompact.getNumRows(), nDegreesOfFreedom);
      compactBlockToFullBlock(jacobian.getJointsInOrder(), jBlockCompact, jFullBlock);

      pointJacobianConvectiveTermCalculator.compute(pointJacobian, pPointVelocity);
      pPointVelocity.scale(-1.0);
      pPointVelocity.add(desiredAccelerationWithRespectToBase);
      DenseMatrix64F pBlock = getMatrixFromList(pList, motionConstraintIndex, selectionMatrix.getNumRows(), 1);
      DenseMatrix64F pPointMatrixVelocity = new DenseMatrix64F(3, 1);
      MatrixTools.setDenseMatrixFromTuple3d(pPointMatrixVelocity, pPointVelocity.getVector(), 0, 0);
      CommonOps.mult(selectionMatrix, pPointMatrixVelocity, pBlock);
      
      MutableDouble weightBlock = getMutableDoubleFromList(weightList, motionConstraintIndex);
      weightBlock.setValue(weight);

      reportPointAccelerationMotionContraint(null, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);
      
      motionConstraintIndex++;
   }
   
   private void reportJointAccelerationMotionContraint(DesiredJointAccelerationCommand desiredJointAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      if (motionConstraintListener != null)
      {
         motionConstraintListener.jointAccelerationMotionConstraintWasAdded(desiredJointAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);
      }
   }
   
   private void reportSpatialAccelerationMotionContraint(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      if (motionConstraintListener != null)
      {
         motionConstraintListener.spatialAccelerationMotionConstraintWasAdded(desiredSpatialAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);
      }
   }
   
   private void reportNullSpaceMultiplierForSpatialAccelerationMotionContraint(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      if (motionConstraintListener != null)
      {
         motionConstraintListener.nullSpaceMultiplierForSpatialAccelerationMotionContraintWasAdded(desiredSpatialAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);
      }
   }
   
   
   
   private void reportPointAccelerationMotionContraint(DesiredPointAccelerationCommand desiredPointAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      if (motionConstraintListener != null)
      {
         motionConstraintListener.pointAccelerationMotionConstraintWasAdded(desiredPointAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);
      }
   }

   public void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand) 
   {
      // (S * J) * vdot = S * (Tdot - Jdot * v)
      GeometricJacobian jacobian = desiredSpatialAccelerationCommand.getJacobian();
      TaskspaceConstraintData taskspaceConstraintData = desiredSpatialAccelerationCommand.getTaskspaceConstraintData();
      double weight = desiredSpatialAccelerationCommand.getWeight();
      
      SpatialAccelerationVector taskSpaceAcceleration = taskspaceConstraintData.getSpatialAcceleration();
      DenseMatrix64F selectionMatrix = taskspaceConstraintData.getSelectionMatrix();
      DenseMatrix64F nullspaceMultipliers = taskspaceConstraintData.getNullspaceMultipliers();

      if (selectionMatrix.getNumRows() > 0)
      {
         RigidBody base = getBase(taskSpaceAcceleration);
         RigidBody endEffector = getEndEffector(taskSpaceAcceleration);

         // TODO: inefficient
         GeometricJacobian baseToEndEffectorJacobian = new GeometricJacobian(base, endEffector, taskSpaceAcceleration.getExpressedInFrame());    // FIXME: garbage, repeated computation
         baseToEndEffectorJacobian.compute();

         // TODO: inefficient
         convectiveTermCalculator.computeJacobianDerivativeTerm(baseToEndEffectorJacobian, convectiveTerm);
         convectiveTerm.getBodyFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getBodyFrame());
         convectiveTerm.getExpressedInFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getExpressedInFrame());
         convectiveTerm.packMatrix(convectiveTermMatrix, 0);

         jBlockCompact.reshape(selectionMatrix.getNumRows(), baseToEndEffectorJacobian.getNumberOfColumns());


         DenseMatrix64F baseToEndEffectorJacobianMatrix = baseToEndEffectorJacobian.getJacobianMatrix();
         DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();

         //TODO: Is this all done correctly here? What does it actually do? Shouldn't the weight block be set too?

         int nullity = nullspaceMultipliers.getNumRows();
         if (nullity > 0)
         {
            DenseMatrix64F zBlock = getMatrixFromList(pList, motionConstraintIndex, nullity, 1);
            DenseMatrix64F nFullBLock = getMatrixFromList(jList, motionConstraintIndex, nullity, nDegreesOfFreedom);

            ////
            
            computeConstraintBlocksForSingularityEscape(selectionMatrix, nullspaceMultipliers, baseToEndEffectorJacobianMatrix, jacobianMatrix, zBlock,
                  nCompactBlock, jBlockCompact);
            
            ///
            
            compactBlockToFullBlock(jacobian.getJointsInOrder(), nCompactBlock, nFullBLock);
            reportNullSpaceMultiplierForSpatialAccelerationMotionContraint(desiredSpatialAccelerationCommand, motionConstraintIndex, nFullBLock, nCompactBlock, zBlock, null);

            motionConstraintIndex++;
         }
         else
         {
            CommonOps.mult(selectionMatrix, baseToEndEffectorJacobianMatrix, jBlockCompact);
         }

         DenseMatrix64F jFullBlock = getMatrixFromList(jList, motionConstraintIndex, jBlockCompact.getNumRows(), nDegreesOfFreedom);
         compactBlockToFullBlock(baseToEndEffectorJacobian.getJointsInOrder(), jBlockCompact, jFullBlock);

//         if (jMatrixToPack != null) jMatrixToPack.setReshape(jFullBlock);
         
         DenseMatrix64F pBlock = getMatrixFromList(pList, motionConstraintIndex, selectionMatrix.getNumRows(), 1);
         taskSpaceAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);
         CommonOps.mult(selectionMatrix, taskSpaceAccelerationMatrix, pBlock);
         CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, pBlock);

//         if (pVectorToPack != null) pVectorToPack.setReshape(pBlock);
         
         MutableDouble weightBlock = getMutableDoubleFromList(weightList, motionConstraintIndex);
         weightBlock.setValue(weight);

         reportSpatialAccelerationMotionContraint(desiredSpatialAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);

         motionConstraintIndex++;
      }
   }

   private static void computeConstraintBlocksForSingularityEscape(DenseMatrix64F selectionMatrix, DenseMatrix64F nullspaceMultipliers,
         DenseMatrix64F baseToEndEffectorJacobianMatrix, DenseMatrix64F jacobianMatrix, DenseMatrix64F zBlock, DenseMatrix64F nCompactBlock, DenseMatrix64F jBlockCompact)
   {
      DenseMatrix64F sJ = new DenseMatrix64F(1, 1); //TODO: Garbage...
      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(SpatialMotionVector.SIZE, true); //TODO: Garbage...

      sJ.reshape(selectionMatrix.getNumRows(), jacobianMatrix.getNumCols());
      CommonOps.mult(selectionMatrix, jacobianMatrix, sJ);
      nullspaceCalculator.setMatrix(sJ, nullspaceMultipliers.getNumRows());
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      DenseMatrix64F iMinusNNT = computeIMinusNNT(nullspace);
      CommonOps.mult(iMinusNNT, baseToEndEffectorJacobianMatrix, jBlockCompact);

      nCompactBlock.reshape(nullspace.getNumCols(), nullspace.getNumRows());
      CommonOps.transpose(nullspace, nCompactBlock);

      zBlock.set(nullspaceMultipliers);
   }

   private static DenseMatrix64F computeIMinusNNT(DenseMatrix64F nullspace)
   {
      DenseMatrix64F iMinusNNT = new DenseMatrix64F(1, 1);    // TODO: make field
      iMinusNNT.reshape(nullspace.getNumRows(), nullspace.getNumRows());
      CommonOps.multOuter(nullspace, iMinusNNT);
      CommonOps.scale(-1.0, iMinusNNT);
      MatrixTools.addDiagonal(iMinusNNT, 1.0);
      return iMinusNNT;
   }

   private void compactBlockToFullBlock(InverseDynamicsJoint[] joints, DenseMatrix64F compactMatrix, DenseMatrix64F fullBlock)
   {
      fullBlock.zero();

      for (InverseDynamicsJoint joint : joints)
      {
         int[] indicesIntoCompactBlock = ScrewTools.computeIndicesForJoint(joints, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         for (int i = 0; i < indicesIntoCompactBlock.length; i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock[i];
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullBlock, 0, fullBlockIndex);
         }
      }
   }

//   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double weight)
   public void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      InverseDynamicsJoint joint = desiredJointAccelerationCommand.getJoint();
      DenseMatrix64F jointAcceleration = desiredJointAccelerationCommand.getDesiredAcceleration();
      double weight = desiredJointAccelerationCommand.getWeight();
      
      CheckTools.checkEquals(joint.getDegreesOfFreedom(), jointAcceleration.getNumRows());
      int[] columnsForJoint = this.columnsForJoints.get(joint);

      if (columnsForJoint != null)    // don't do anything for joints that are not in the list
      {
         DenseMatrix64F jBlock = getMatrixFromList(jList, motionConstraintIndex, jointAcceleration.getNumRows(), nDegreesOfFreedom);
         jBlock.zero();

         for (int i = 0; i < jointAcceleration.getNumRows(); i++)
         {
            jBlock.set(i, columnsForJoint[i], 1.0);
         }

         DenseMatrix64F pBlock = getMatrixFromList(pList, motionConstraintIndex, jointAcceleration.getNumRows(), 1);
         pBlock.set(jointAcceleration);

         MutableDouble weightBlock = getMutableDoubleFromList(weightList, motionConstraintIndex);
         weightBlock.setValue(weight);

         reportJointAccelerationMotionContraint(desiredJointAccelerationCommand, motionConstraintIndex, jBlock, jBlock, pBlock, weightBlock);

         motionConstraintIndex++;
      }
   }

   public void setDesiredPointAcceleration(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase,
           double weight)
   {
      pointJacobian.set(jacobian, bodyFixedPoint);
      pointJacobian.compute();
      desiredAccelerationWithRespectToBase.changeFrame(jacobian.getBaseFrame());

      DenseMatrix64F pointJacobianMatrix = pointJacobian.getJacobianMatrix();
      DenseMatrix64F jFullBlock = getMatrixFromList(jList, motionConstraintIndex, pointJacobianMatrix.getNumRows(), nDegreesOfFreedom);
      compactBlockToFullBlock(jacobian.getJointsInOrder(), pointJacobianMatrix, jFullBlock);

      pointJacobianConvectiveTermCalculator.compute(pointJacobian, pPointVelocity);
      pPointVelocity.scale(-1.0);
      pPointVelocity.add(desiredAccelerationWithRespectToBase);
      DenseMatrix64F pBlock = getMatrixFromList(pList, motionConstraintIndex, pointJacobianMatrix.getNumRows(), 1);
      MatrixTools.setDenseMatrixFromTuple3d(pBlock, pPointVelocity.getVector(), 0, 0);

      MutableDouble weightBlock = getMutableDoubleFromList(weightList, motionConstraintIndex);
      weightBlock.setValue(weight);

      reportPointAccelerationMotionContraint(null, motionConstraintIndex, jFullBlock, pointJacobianMatrix, pBlock, weightBlock);

      motionConstraintIndex++;
   }

   public void compute()
   {
      assembleEquation(jList, pList, motionConstraintIndex, j, p);
      assembleWeightMatrix(weightList, jList, motionConstraintIndex, w);
   }
   
   public void getMotionConstraintJMatrixPVectorAndWeight(int motionConstraintIndex, DenseMatrix64F jToPack, DenseMatrix64F pToPack, MutableDouble wToPack)
   {
      jToPack.setReshape(jList.get(motionConstraintIndex));
      pToPack.setReshape(pList.get(motionConstraintIndex));
      wToPack.setValue(weightList.get(motionConstraintIndex));
   }

   private void assembleEquation(List<DenseMatrix64F> matrixList, List<DenseMatrix64F> vectorList, int size, DenseMatrix64F matrix, DenseMatrix64F vector)
   {
      if (matrixList.size() != vectorList.size())
         throw new RuntimeException("sizes not equal");

      int nRows = 0;
      for (int i = 0; i < size; i++)
      {
         nRows += matrixList.get(i).getNumRows();
      }

      matrix.reshape(nRows, nDegreesOfFreedom);
      vector.reshape(nRows, 1);
      matrix.zero();
      vector.zero();

      int rowNumber = 0;
      for (int i = 0; i < size; i++)
      {
         DenseMatrix64F matrixBlock = matrixList.get(i);
         CommonOps.insert(matrixBlock, matrix, rowNumber, 0);

         DenseMatrix64F vectorBlock = vectorList.get(i);
         CommonOps.insert(vectorBlock, vector, rowNumber, 0);

         rowNumber += matrixBlock.getNumRows();
      }
   }

   private void assembleWeightMatrix(List<MutableDouble> weightList, List<DenseMatrix64F> jacobianList, int nConstraints, DenseMatrix64F weightMatrix)
   {
      CheckTools.checkEquals(weightList.size(), jacobianList.size());

      int size = 0;
      for (int i = 0; i < nConstraints; i++)
      {
         size += jacobianList.get(i).getNumRows();
      }

      weightMatrix.reshape(size, size);
      weightMatrix.zero();
      int matrixIndex = 0;
      for (int i = 0; i < nConstraints; i++)
      {
         double weight = weightList.get(i).doubleValue();
         int blockSize = jacobianList.get(i).getNumRows();
         for (int blockIndex = 0; blockIndex < blockSize; blockIndex++)
         {
            weightMatrix.set(matrixIndex, matrixIndex, weight);
            matrixIndex++;
         }
      }
   }

   public DenseMatrix64F getJacobian()
   {
      return j;
   }

   public DenseMatrix64F getRightHandSide()
   {
      return p;
   }

   public DenseMatrix64F getWeightMatrix()
   {
      return w;
   }

   private static DenseMatrix64F getMatrixFromList(List<DenseMatrix64F> matrixList, int index, int nRows, int nColumns)
   {
      for (int i = matrixList.size(); i <= index; i++)
      {
         matrixList.add(new DenseMatrix64F(1, 1));
      }

      DenseMatrix64F ret = matrixList.get(index);
      ret.reshape(nRows, nColumns);

      return ret;
   }

   private static MutableDouble getMutableDoubleFromList(List<MutableDouble> mutableDoubleList, int index)
   {
      for (int i = mutableDoubleList.size(); i <= index; i++)
      {
         mutableDoubleList.add(new MutableDouble());
      }

      MutableDouble ret = mutableDoubleList.get(index);

      return ret;
   }

   private RigidBody getBase(SpatialAccelerationVector taskSpaceAcceleration)
   {
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         RigidBody predecessor = joint.getPredecessor();
         if (predecessor.getBodyFixedFrame() == taskSpaceAcceleration.getBaseFrame())
            return predecessor;
      }

      throw new RuntimeException("Base for " + taskSpaceAcceleration + " could not be determined");
   }

   private RigidBody getEndEffector(SpatialAccelerationVector taskSpaceAcceleration)
   {
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         RigidBody successor = joint.getSuccessor();
         if (successor.getBodyFixedFrame() == taskSpaceAcceleration.getBodyFrame())
            return successor;
      }

      throw new RuntimeException("End effector for " + taskSpaceAcceleration + " could not be determined");
   }
}
