package us.ihmc.commonWalkingControlModules.momentumBasedController;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.ops.CommonOps;
import us.ihmc.utilities.CheckTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * @author twan
 *         Date: 4/22/13
 */
public class MomentumSolver3 implements MomentumSolverInterface
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

   private final DenseMatrix64F A;
   private final DenseMatrix64F b;

   private final DenseMatrix64F Jp = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F pp = new DenseMatrix64F(1, 1);

   private final List<DenseMatrix64F> JpList = new ArrayList<DenseMatrix64F>();
   private final List<DenseMatrix64F> ppList = new ArrayList<DenseMatrix64F>();


   private final DenseMatrix64F vdot;

   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();

   private final DenseMatrix64F adotV = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F v;
   private final DenseMatrix64F hdot = new DenseMatrix64F(Momentum.SIZE, 1);

   private final int nDegreesOfFreedom;

   private final LinearSolver<DenseMatrix64F> solver;

   public MomentumSolver3(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
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

      nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      this.A = new DenseMatrix64F(SpatialMotionVector.SIZE, nDegreesOfFreedom);
      this.b = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

      this.vdot = new DenseMatrix64F(nDegreesOfFreedom, 1);
      this.v = new DenseMatrix64F(nDegreesOfFreedom, 1);

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         columnsForJoints.put(joint, ScrewTools.computeIndicesForJoint(jointsInOrder, joint));
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
      JpList.clear();
      ppList.clear();
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      CheckTools.checkEquals(joint.getDegreesOfFreedom(), jointAcceleration.getNumRows());
      int[] columnsForJoint = this.columnsForJoints.get(joint);

      DenseMatrix64F JpBlock = new DenseMatrix64F(joint.getDegreesOfFreedom(), nDegreesOfFreedom);
      for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
      {
         JpBlock.set(i, columnsForJoint[i], 1.0);
      }

      DenseMatrix64F ppBlock = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
      ppBlock.set(jointAcceleration);

      JpList.add(JpBlock);
      ppList.add(ppBlock);
   }

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F taskSpaceAccelerationMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

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

      DenseMatrix64F JpBlockCompact = new DenseMatrix64F(selectionMatrix.getNumRows(), baseToEndEffectorJacobian.getNumberOfColumns()); // TODO: garbage
      CommonOps.mult(selectionMatrix, baseToEndEffectorJacobian.getJacobianMatrix(), JpBlockCompact);

      DenseMatrix64F JpFullBlock = new DenseMatrix64F(JpBlockCompact.getNumRows(), nDegreesOfFreedom); // TODO: garbage

      for (InverseDynamicsJoint joint : baseToEndEffectorJacobian.getJointsInOrder())
      {
         int[] indicesIntoCompactBlock = ScrewTools.computeIndicesForJoint(baseToEndEffectorJacobian.getJointsInOrder(), joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         for (int i = 0; i < indicesIntoCompactBlock.length; i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock[i];
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(JpBlockCompact, 0, JpBlockCompact.getNumRows(), compactBlockIndex, compactBlockIndex + 1, JpFullBlock, 0, fullBlockIndex);
         }
      }
      JpList.add(JpFullBlock);

      DenseMatrix64F ppBlock = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      taskSpaceAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);
      CommonOps.mult(selectionMatrix, taskSpaceAccelerationMatrix, ppBlock);
      CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, ppBlock);
      ppList.add(ppBlock);
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

   private final DenseMatrix64F sTranspose = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F JpPlus = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F JpPluspp = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F P = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F AP = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F bMinusAJPlusP = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F APPlusbMinusAJpPluspp = new DenseMatrix64F(1, 1);


   public void solve(DenseMatrix64F accelerationSubspace, DenseMatrix64F accelerationMultipliers, DenseMatrix64F momentumSubspace,
                     DenseMatrix64F momentumMultipliers)
   {
      // TODO: subspaces
      CommonOps.sub(momentumMultipliers, adotV, b);

      // add root joint constraint (just call the same function)
      sTranspose.reshape(accelerationSubspace.getNumCols(), accelerationSubspace.getNumRows());
      CommonOps.transpose(accelerationSubspace, sTranspose);

      // assemble Jp, pp
      assemblePrimaryMotionConstraints();

      // J+
      JpPlus.reshape(Jp.getNumCols(), Jp.getNumRows());
      solver.setA(Jp);
      solver.invert(JpPlus);

      // J+p
      JpPluspp.reshape(JpPlus.getNumRows(), pp.getNumCols());
      CommonOps.mult(JpPlus, pp, JpPluspp);

      // bMinusAJPlusp
      bMinusAJPlusP.set(b);
      CommonOps.multAdd(-1.0, centroidalMomentumMatrix.getMatrix(), JpPluspp, bMinusAJPlusP);


      // P
      P.reshape(JpPlus.getNumRows(), Jp.getNumCols());
      CommonOps.setIdentity(P);
      CommonOps.multAdd(-1.0, JpPlus, Jp, P);

      // AP
      AP.reshape(centroidalMomentumMatrix.getMatrix().getNumRows(), P.getNumCols());
      CommonOps.mult(centroidalMomentumMatrix.getMatrix(), P, AP);


      // APPlusbMinusAJpPluspp
      APPlusbMinusAJpPluspp.reshape(AP.getNumCols(), bMinusAJPlusP.getNumCols());
      solver.setA(AP);
      solver.solve(bMinusAJPlusP, APPlusbMinusAJpPluspp);

      // vdot
      vdot.set(JpPluspp);
      CommonOps.multAdd(P, APPlusbMinusAJpPluspp, vdot);

      ScrewTools.setDesiredAccelerations(jointsInOrder, vdot);
   }

   private void assemblePrimaryMotionConstraints()
   {
      if (JpList.size() != ppList.size())
         throw new RuntimeException("JpList.size() != ppList.size()");

      int jpRows = 0;
      for (int i = 0; i < JpList.size(); i++)
      {
         jpRows += JpList.get(i).getNumRows();
      }
      Jp.reshape(jpRows, nDegreesOfFreedom);
      pp.reshape(jpRows, 1);

      int rowNumber = 0;
      for (int i = 0; i < JpList.size(); i++)
      {
         DenseMatrix64F JpBlock = JpList.get(i);
         CommonOps.insert(JpBlock, Jp, rowNumber, 0);

         DenseMatrix64F ppBlock = ppList.get(i);
         CommonOps.insert(ppBlock, pp, rowNumber, 0);

         rowNumber += JpBlock.getNumRows();
      }
   }

   public void solve(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData momentumRateOfChangeData)
   {
      solve(rootJointAccelerationData.getAccelerationSubspace(), rootJointAccelerationData.getAccelerationMultipliers(),
            momentumRateOfChangeData.getMomentumSubspace(), momentumRateOfChangeData.getMomentumMultipliers());
   }

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
