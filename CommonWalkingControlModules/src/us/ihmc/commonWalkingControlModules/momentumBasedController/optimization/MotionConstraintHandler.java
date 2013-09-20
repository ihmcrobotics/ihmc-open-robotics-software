package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.apache.commons.lang.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SingularOps;

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
   private final MotionConstraintSingularityEscapeHandler motionConstraintSingularityEscapeHandler = new MotionConstraintSingularityEscapeHandler();

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
//      setDesiredSpatialAccelerationOld(desiredSpatialAccelerationCommand);
      setDesiredSpatialAccelerationNew(desiredSpatialAccelerationCommand);
   }
   
   public void setDesiredSpatialAccelerationNew(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand) 
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


         DenseMatrix64F baseToEndEffectorJacobianMatrix = new DenseMatrix64F(baseToEndEffectorJacobian.getJacobianMatrix()); //TODO: Garbage
         DenseMatrix64F jacobianMatrix = new DenseMatrix64F(jacobian.getJacobianMatrix()); //TODO: Garbage

         int nullity = nullspaceMultipliers.getNumRows();
         if (nullity > 0)
         {
            // Take the singular value decomposition of the leg Jacobian to find the approximate nullspace.
            
            SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(jacobianMatrix.getNumRows(), jacobianMatrix.getNumCols(), true, true, false);
            svd.decompose(jacobianMatrix);
            
            DenseMatrix64F matrixUTranspose = new DenseMatrix64F(jacobianMatrix.getNumRows(), jacobianMatrix.getNumRows());
            DenseMatrix64F matrixW = new DenseMatrix64F(jacobianMatrix.getNumRows(), jacobianMatrix.getNumCols());
            DenseMatrix64F matrixVTranspose = new DenseMatrix64F(jacobianMatrix.getNumCols(), jacobianMatrix.getNumCols());
            svd.getU(matrixUTranspose, true);
            svd.getW(matrixW);
            svd.getV(matrixVTranspose, true); 
            
            SingularOps.descendingOrder(matrixUTranspose, true, matrixW, matrixVTranspose, true);
            
//            DenseMatrix64F matrixCTranspose = new DenseMatrix64F(jacobianMatrix.getNumCols() - nullity, jacobianMatrix.getNumRows());
            DenseMatrix64F matrixCTranspose = CommonOps.extract(matrixUTranspose, 0, jacobianMatrix.getNumCols() - nullity, 0, jacobianMatrix.getNumRows());
            
            DenseMatrix64F matrixCTransposeJ = new DenseMatrix64F(matrixCTranspose.getNumRows(), baseToEndEffectorJacobianMatrix.getNumCols());
            CommonOps.mult(matrixCTranspose, baseToEndEffectorJacobianMatrix, matrixCTransposeJ);
            
            taskSpaceAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);
            
            DenseMatrix64F temp = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
            CommonOps.mult(selectionMatrix, taskSpaceAccelerationMatrix, temp);
            CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, temp);

            DenseMatrix64F cTransposeP = new DenseMatrix64F(matrixCTranspose.getNumRows(), temp.getNumCols());
            CommonOps.mult(matrixCTranspose, temp, cTransposeP);
            
            DenseMatrix64F pBlock = getMatrixFromList(pList, motionConstraintIndex, selectionMatrix.getNumRows(), 1);
            CommonOps.extract(cTransposeP, 0, cTransposeP.getNumRows(), 0, cTransposeP.getNumCols(), pBlock, 0, 0);
            CommonOps.extract(nullspaceMultipliers, 0, nullspaceMultipliers.getNumRows(), 0, nullspaceMultipliers.getNumCols(), pBlock, cTransposeP.getNumRows(), 0);

            
            //TODO: Figure out how to deal with selection matrix here.
//            DenseMatrix64F selectionMatrixTimesCTransposeJ = new DenseMatrix64F(selectionMatrix.getNumRows(), matrixCTransposeJ.getNumCols());
//            CommonOps.mult(selectionMatrix, matrixCTransposeJ, selectionMatrixTimesCTransposeJ);

         
            for (int i=0; i<jBlockCompact.getNumRows(); i++)
            {
               for (int j=0; j<jBlockCompact.getNumCols(); j++)
               {
                  jBlockCompact.set(i, j, 0.0);
               }
            }
            
            DenseMatrix64F nullSpaceMatrixNTranspose = new DenseMatrix64F(nullity, matrixVTranspose.getNumCols());
            CommonOps.extract(matrixVTranspose, matrixVTranspose.getNumRows() - nullity, matrixVTranspose.getNumRows(), 0, matrixVTranspose.getNumCols(), nullSpaceMatrixNTranspose, 0, 0);  
            System.out.println("nullSpaceMatrixNTranspose = " + nullSpaceMatrixNTranspose);
            NullspaceCalculator.makeLargestComponentInEachColumnPositive(nullSpaceMatrixNTranspose);
            System.out.println("nullSpaceMatrixNTranspose = " + nullSpaceMatrixNTranspose);

            //TODO: Fix the null space stuff. Verify that the three things added are correct. Figure out why qdd is way off from desired.
            CommonOps.extract(matrixCTransposeJ, 0, matrixCTransposeJ.getNumRows(), 0, matrixCTransposeJ.getNumCols(), jBlockCompact, 0, 0);
            CommonOps.extract(nullSpaceMatrixNTranspose, 0, nullSpaceMatrixNTranspose.getNumRows(), 0, nullSpaceMatrixNTranspose.getNumCols(), jBlockCompact, matrixCTransposeJ.getNumRows(), 6);  //Hack 6
            
            System.out.println("jBlockCompact = " + jBlockCompact);
            
//            jBlockCompact.set(5, 8, -0.5);
//            jBlockCompact.set(5, 9, 1.0);
//            jBlockCompact.set(5, 10, -0.5);
            
//            System.out.println("jBlockCompact = " + jBlockCompact);
            
            
            DenseMatrix64F jFullBlock = getMatrixFromList(jList, motionConstraintIndex, jBlockCompact.getNumRows(), nDegreesOfFreedom);
            compactBlockToFullBlock(baseToEndEffectorJacobian.getJointsInOrder(), jBlockCompact, jFullBlock);
            
            MutableDouble weightBlock = getMutableDoubleFromList(weightList, motionConstraintIndex);
            weightBlock.setValue(weight);

            reportSpatialAccelerationMotionContraint(desiredSpatialAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);

            motionConstraintIndex++;
         }
         else
         {
            CommonOps.mult(selectionMatrix, baseToEndEffectorJacobianMatrix, jBlockCompact);
            
            DenseMatrix64F pBlock = getMatrixFromList(pList, motionConstraintIndex, selectionMatrix.getNumRows(), 1);
            taskSpaceAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);
            CommonOps.mult(selectionMatrix, taskSpaceAccelerationMatrix, pBlock);
            CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, pBlock);
            
            DenseMatrix64F jFullBlock = getMatrixFromList(jList, motionConstraintIndex, jBlockCompact.getNumRows(), nDegreesOfFreedom);
            compactBlockToFullBlock(baseToEndEffectorJacobian.getJointsInOrder(), jBlockCompact, jFullBlock);
            
            MutableDouble weightBlock = getMutableDoubleFromList(weightList, motionConstraintIndex);
            weightBlock.setValue(weight);

            reportSpatialAccelerationMotionContraint(desiredSpatialAccelerationCommand, motionConstraintIndex, jFullBlock, jBlockCompact, pBlock, weightBlock);

            motionConstraintIndex++;
         }
      }
   }
   
   
   public void setDesiredSpatialAccelerationOld(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand) 
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
            
            // Handle singularity escape:
            motionConstraintSingularityEscapeHandler.computeConstraintBlocksForSingularityEscape(selectionMatrix, nullspaceMultipliers, baseToEndEffectorJacobianMatrix, jacobianMatrix, zBlock,
                  nCompactBlock, jBlockCompact);
            
            
//            System.out.println("Singularity escape. nCompactBlock = " + nCompactBlock);
//            System.out.println("Singularity escape. zBlock = " + zBlock);
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
