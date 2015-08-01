package us.ihmc.commonWalkingControlModules.momentumBasedController;

import gnu.trove.list.array.TIntArrayList;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;


import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.DesiredJointAccelerationCalculator;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools;


/**
 * @author twan
 *         Date: 4/22/13
 */
public class MomentumSolver2 implements MomentumSolverInterface
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix;    // to make numerical differentiation rewindable

   private final InverseDynamicsJoint rootJoint; // TODO: make this not be special
   private final InverseDynamicsJoint[] jointsInOrder;

   private final double controlDT;

   private final DenseMatrix64F AJ;
   private final DenseMatrix64F bp;
   private final DenseMatrix64F vdot;
   private final DenseMatrix64F nullspacePart;

   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();

   private final DenseMatrix64F adotV = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F v;
   private final DenseMatrix64F hdot = new DenseMatrix64F(Momentum.SIZE, 1);
   private final LinearSolver<DenseMatrix64F> jacobianSolver;

   private int ajIndex = 0;
   private final int nDegreesOfFreedom;

   private final LinearSolver<DenseMatrix64F> solver;
   private final Map<GeometricJacobian, TaskspaceConstraintData> taskSpaceConstraintMap = new LinkedHashMap<GeometricJacobian, TaskspaceConstraintData>();

   public MomentumSolver2(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
                          LinearSolver<DenseMatrix64F> jacobianSolver, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      this.jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
            centroidalMomentumMatrix.getMatrix().getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
            centroidalMomentumMatrix.getMatrix().getNumCols());
      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      this.controlDT = controlDT;
      this.jacobianSolver = jacobianSolver;

      nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      this.AJ = new DenseMatrix64F(nDegreesOfFreedom, nDegreesOfFreedom);
      this.bp = new DenseMatrix64F(nDegreesOfFreedom, 1);
      this.vdot = new DenseMatrix64F(nDegreesOfFreedom, 1);
      this.nullspacePart = new DenseMatrix64F(nDegreesOfFreedom, 1);
      this.v = new DenseMatrix64F(nDegreesOfFreedom, 1);

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();
         
         columnsForJoints.put(joint, indices);
      }

      solver = LinearSolverFactory.pseudoInverse(true);

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize()
   {
      centroidalMomentumMatrix.compute();
      previousCentroidalMomentumMatrix.set(centroidalMomentumMatrix.getMatrix());
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
   }

   public void reset()
   {
      ajIndex = 0;
      AJ.zero();
      bp.zero();
      nullspacePart.zero();
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      MathTools.checkIfEqual(joint.getDegreesOfFreedom(), jointAcceleration.getNumRows());
      int[] columnsForJoint = this.columnsForJoints.get(joint);
      for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
      {
         AJ.set(ajIndex + i, columnsForJoint[i], 1.0);
      }

      CommonOps.insert(jointAcceleration, bp, ajIndex, 0);
      ajIndex += joint.getDegreesOfFreedom();
   }

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F taskSpaceAccelerationMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F JBlock = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F pBlock = new DenseMatrix64F(1, 1);
   private final TIntArrayList indicesIntoBlock = new TIntArrayList();

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      setDesiredSpatialAcceleration(jacobian.getJointsInOrder(), jacobian, taskspaceConstraintData);
   }

   public void setDesiredSpatialAcceleration(InverseDynamicsJoint[] constrainedJoints, GeometricJacobian jacobian, TaskspaceConstraintData
         taskspaceConstraintData)
   {
      // (S * J) * vdot = S * (Tdot - Jdot * v)

      SpatialAccelerationVector taskSpaceAcceleration = taskspaceConstraintData.getSpatialAcceleration();
      DenseMatrix64F selectionMatrix = taskspaceConstraintData.getSelectionMatrix();

      RigidBody base = getBase(taskSpaceAcceleration);
      RigidBody endEffector = getEndEffector(taskSpaceAcceleration);

      // TODO: inefficient
      GeometricJacobian baseToEndEffectorJacobian = new GeometricJacobian(base, endEffector, taskSpaceAcceleration.getExpressedInFrame());    // FIXME: garbage, repeated computation
      baseToEndEffectorJacobian.compute();

      // TODO: inefficient
      DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(baseToEndEffectorJacobian, null);    // TODO: garbage
      desiredJointAccelerationCalculator.computeJacobianDerivativeTerm(convectiveTerm);
      convectiveTerm.getBodyFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getBodyFrame());
      convectiveTerm.getExpressedInFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getExpressedInFrame());
      convectiveTerm.packMatrix(convectiveTermMatrix, 0);

      JBlock.reshape(selectionMatrix.getNumRows(), baseToEndEffectorJacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, baseToEndEffectorJacobian.getJacobianMatrix(), JBlock);

      pBlock.reshape(selectionMatrix.getNumRows(), 1);
      taskSpaceAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);
      CommonOps.mult(selectionMatrix, taskSpaceAccelerationMatrix, pBlock);
      CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, pBlock);

      for (InverseDynamicsJoint joint : baseToEndEffectorJacobian.getJointsInOrder())
      {
         //old garbage-full way
//         int[] indicesIntoBlock = ScrewTools.computeIndicesForJoint(baseToEndEffectorJacobian.getJointsInOrder(), joint); 
         indicesIntoBlock.clear();
         ScrewTools.computeIndexForJoint(baseToEndEffectorJacobian.getJointsInOrder(), indicesIntoBlock, joint);
         int[] indicesIntoBigMatrix = columnsForJoints.get(joint);

         for (int i = 0; i < indicesIntoBlock.size(); i++)
         {
            int blockIndex = indicesIntoBlock.get(i);
            int bigMatrixIndex = indicesIntoBigMatrix[i];
            CommonOps.extract(JBlock, 0, JBlock.getNumRows(), blockIndex, blockIndex + 1, AJ, ajIndex, bigMatrixIndex);
         }
      }

      CommonOps.insert(pBlock, bp, ajIndex, 0);

      taskSpaceConstraintMap.put(jacobian, taskspaceConstraintData);

      ajIndex +=  JBlock.getNumRows();
   }

   public void compute()
   {
      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
            controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      CommonOps.mult(centroidalMomentumMatrixDerivative, v, adotV);
   }

   private final DenseMatrix64F T = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F alpha1 = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F N = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F beta1 = new DenseMatrix64F(1, 1);

   public void solve(SpatialForceVector momentumRateOfChange)
   {
      T.reshape(SpatialMotionVector.SIZE, 0);
      alpha1.reshape(T.getNumCols(), 1);
      N.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
      beta1.reshape(N.getNumCols(), 1);

      CommonOps.setIdentity(N);
      momentumRateOfChange.packMatrix(beta1);

      solve(T, alpha1, N, beta1);
   }

   private final DenseMatrix64F NA = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F b = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F sTranspose = new DenseMatrix64F(1, 1);
   public void solve(DenseMatrix64F accelerationSubspace, DenseMatrix64F accelerationMultipliers, DenseMatrix64F momentumSubspace,
                     DenseMatrix64F momentumMultipliers)
   {
      sTranspose.reshape(accelerationSubspace.getNumCols(), accelerationSubspace.getNumRows());
      CommonOps.transpose(accelerationSubspace, sTranspose);

      int[] indicesIntoBigMatrix = columnsForJoints.get(rootJoint);
      for (int i = 0; i < sTranspose.getNumCols(); i++)
      {
         int bigMatrixIndex = indicesIntoBigMatrix[i];
         CommonOps.extract(sTranspose, 0, sTranspose.getNumRows(), i, i + 1, AJ, ajIndex, bigMatrixIndex);
      }

      CommonOps.insert(accelerationMultipliers, bp, ajIndex, 0);
      // don't increment ajIndex so that we can overwrite the last couple of columns to solve for other cases too
//      ajIndex += sTranspose.getNumRows();

      NA.reshape(momentumSubspace.getNumCols(), centroidalMomentumMatrix.getMatrix().getNumCols());
      CommonOps.multTransA(momentumSubspace, centroidalMomentumMatrix.getMatrix(), NA);
      CommonOps.insert(NA, AJ, ajIndex + sTranspose.getNumRows(), 0);

      // N * (hdot - Adotv)
      b.reshape(momentumMultipliers.getNumRows(), 1);
      CommonOps.multTransA(momentumSubspace, adotV, b);
      CommonOps.changeSign(b);
      CommonOps.addEquals(b, momentumMultipliers);
      CommonOps.insert(b, bp, ajIndex + sTranspose.getNumRows(), 0);

      // don't increment ajIndex so that we can overwrite the last couple of columns to solve for other cases too
//      ajIndex += momentumSubspace.getNumRows();

      solver.setA(AJ);
      solver.solve(bp, vdot);

//      setNullspaceComponents(vdot);

      ScrewTools.setDesiredAccelerations(jointsInOrder, vdot);
   }

//   private void setNullspaceComponents(DenseMatrix64F vdot)
//   {
//      for (GeometricJacobian jacobian : taskSpaceConstraintMap.keySet())
//      {
//         TaskspaceConstraintData taskspaceConstraintData = taskSpaceConstraintMap.get(jacobian);
//
//         DenseMatrix64F nullspaceMultiplier = taskspaceConstraintData.getNullspaceMultipliers();
//         int nullity = nullspaceMultiplier.getNumRows();
//
//         if (nullity > 0)
//         {
//            DenseMatrix64F selectionMatrix = taskspaceConstraintData.getSelectionMatrix();
//
//            jacobian.compute();
//            DenseMatrix64F reducedJacobianMatrix = new DenseMatrix64F(selectionMatrix.getNumRows(), jacobian.getNumberOfColumns());
//            CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), reducedJacobianMatrix);
//
//            NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(reducedJacobianMatrix.getNumRows(), true);
//            nullspaceCalculator.setMatrix(reducedJacobianMatrix, nullity);
//
//            int[] columnsForJoints = ScrewTools.computeIndicesForJoint(jointsInOrder, jacobian.getJointsInOrder());
//            DenseMatrix64F vdotPart = new DenseMatrix64F(columnsForJoints.length, 1);
//            for (int i = 0; i < columnsForJoints.length; i++)
//            {
//               vdotPart.set(i, 0, vdot.get(columnsForJoints[i], 0));
//            }
//
//            nullspaceCalculator.removeNullspaceComponent(vdotPart);
//            nullspaceCalculator.addNullspaceComponent(vdotPart, nullspaceMultiplier);
//
//            for (int i = 0; i < columnsForJoints.length; i++)
//            {
//               vdot.set(columnsForJoints[i], 0, vdotPart.get(i, 0));
//            }
//         }
//      }
//   }

   public void getRateOfChangeOfMomentum(SpatialForceVector rateOfChangeOfMomentumToPack)
   {
      CommonOps.mult(centroidalMomentumMatrix.getMatrix(), vdot, hdot);
      CommonOps.addEquals(hdot, adotV);
      rateOfChangeOfMomentumToPack.set(centroidalMomentumMatrix.getReferenceFrame(), hdot);
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
