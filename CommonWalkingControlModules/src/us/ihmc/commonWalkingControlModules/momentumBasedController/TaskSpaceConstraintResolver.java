package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.NullspaceCalculator;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.DesiredJointAccelerationCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class TaskSpaceConstraintResolver
{
   private final ReferenceFrame rootJointFrame;
   private final RigidBody rootJointPredecessor;
   private final RigidBody rootJointSuccessor;

   private final InverseDynamicsJoint[] jointsInOrder;
   private final TwistCalculator twistCalculator;
   private final NullspaceCalculator nullspaceCalculator;
   private final LinearSolver<DenseMatrix64F> jacobianSolver;

   private final DenseMatrix64F aTaskSpace;
   private final DenseMatrix64F vdotTaskSpace;
   private final DenseMatrix64F cTaskSpace;
   private final DenseMatrix64F taskSpaceAccelerationMatrix;
   private final DenseMatrix64F sJInverse;
   private final DenseMatrix64F sJ;

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final Twist twistOfCurrentWithRespectToNew = new Twist();
   private final Twist twistOfBodyWithRespectToBase = new Twist();

   // LinkedHashMaps so that order of computation is defined, to make rewindability checks using reflection easier
   private final LinkedHashMap<MechanismGeometricJacobian, DenseMatrix64F> dMap = new LinkedHashMap<MechanismGeometricJacobian, DenseMatrix64F>();
   private final LinkedHashMap<MechanismGeometricJacobian, DenseMatrix64F> jacobianInverseMap = new LinkedHashMap<MechanismGeometricJacobian, DenseMatrix64F>();

   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public TaskSpaceConstraintResolver(SixDoFJoint rootJoint, InverseDynamicsJoint[] jointsInOrder, TwistCalculator twistCalculator,
                                      NullspaceCalculator nullspaceCalculator, LinearSolver<DenseMatrix64F> jacobianSolver)
   {
      rootJointFrame = rootJoint.getFrameAfterJoint();
      rootJointPredecessor = rootJoint.getPredecessor();
      rootJointSuccessor = rootJoint.getSuccessor();
      this.jointsInOrder = jointsInOrder;
      this.twistCalculator = twistCalculator;
      this.nullspaceCalculator = nullspaceCalculator;
      this.jacobianSolver = jacobianSolver;

      int size = Momentum.SIZE;
      this.aTaskSpace = new DenseMatrix64F(size, size);    // reshaped
      this.vdotTaskSpace = new DenseMatrix64F(size, 1);    // reshaped
      this.cTaskSpace = new DenseMatrix64F(size, 1);    // reshaped
      this.taskSpaceAccelerationMatrix = new DenseMatrix64F(size, 1);
      this.sJInverse = new DenseMatrix64F(size, size);    // reshaped
      this.sJ = new DenseMatrix64F(size, size);    // reshaped
   }

   public void reset()
   {
      dMap.clear();
      jacobianInverseMap.clear();
   }

   public void handleTaskSpaceAccelerations(DenseMatrix64F aHatRoot, DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrix,
           MechanismGeometricJacobian jacobian, SpatialAccelerationVector taskSpaceAcceleration, DenseMatrix64F nullspaceMultiplier,
           DenseMatrix64F selectionMatrix)
   {
      assert(selectionMatrix.getNumCols() == SpatialAccelerationVector.SIZE);

      if (taskSpaceAcceleration.getExpressedInFrame() != rootJointFrame)
      {
         twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, rootJointSuccessor, jacobian.getEndEffector());
         twistCalculator.packRelativeTwist(twistOfBodyWithRespectToBase, rootJointPredecessor, jacobian.getEndEffector());
         taskSpaceAcceleration.changeFrame(rootJointSuccessor.getBodyFixedFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
         taskSpaceAcceleration.changeFrameNoRelativeMotion(rootJointFrame);
      }

      // taskSpaceAcceleration
      taskSpaceAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);

      // J
      jacobian.changeFrame(rootJointFrame);
      jacobian.compute();
      sJ.reshape(selectionMatrix.getNumRows(), jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), sJ);

      // aTaskSpace
      int[] columnIndices = ScrewTools.computeIndicesForJoint(jointsInOrder, jacobian.getJointsInOrder());
      aTaskSpace.reshape(aTaskSpace.getNumRows(), columnIndices.length);
      MatrixTools.extractColumns(centroidalMomentumMatrix, aTaskSpace, columnIndices);

      // convectiveTerm
      DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, null);    // TODO: garbage
      desiredJointAccelerationCalculator.computeJacobianDerivativeTerm(convectiveTerm);
      twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, rootJointSuccessor, jacobian.getEndEffector());
      twistCalculator.packRelativeTwist(twistOfBodyWithRespectToBase, jacobian.getBase(), jacobian.getEndEffector());
      convectiveTerm.changeFrame(rootJointSuccessor.getBodyFixedFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
      convectiveTerm.changeFrameNoRelativeMotion(rootJointFrame);
      convectiveTerm.getBodyFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getBodyFrame());
      convectiveTerm.getExpressedInFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getExpressedInFrame());
      convectiveTerm.packMatrix(convectiveTermMatrix, 0);

      sJInverse.reshape(sJ.getNumCols(), sJ.getNumCols());
      DenseMatrix64F d = new DenseMatrix64F(sJInverse.getNumRows(), 1);    // TODO: garbage

      // JInverse
      jacobianSolver.setA(sJ);
      jacobianSolver.invert(sJInverse);

      int nullity = nullspaceMultiplier.getNumRows();
      if (nullity > 0)
      {
         nullspaceCalculator.setMatrix(sJ, nullity);    // TODO: check if this works for S != I6
         nullspaceCalculator.removeNullspaceComponent(sJInverse);
      }

      // cTaskSpace
      cTaskSpace.reshape(selectionMatrix.getNumRows(), 1);
      cTaskSpace.zero();
      CommonOps.multAdd(1.0, selectionMatrix, taskSpaceAccelerationMatrix, cTaskSpace);
      CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, cTaskSpace);

      // d
      CommonOps.mult(sJInverse, cTaskSpace, d);

      if (nullity > 0)
      {
         nullspaceCalculator.addNullspaceComponent(d, nullspaceMultiplier);
      }

      // update aHatRoot
      DenseMatrix64F sJInverseS = new DenseMatrix64F(sJInverse.getNumRows(), selectionMatrix.getNumCols());    // TODO: garbage
      CommonOps.mult(sJInverse, selectionMatrix, sJInverseS);
      CommonOps.multAdd(-1.0, aTaskSpace, sJInverseS, aHatRoot);

      // update b
      CommonOps.multAdd(-1.0, aTaskSpace, d, b);

      jacobianInverseMap.put(jacobian, sJInverseS);
      dMap.put(jacobian, d);
   }

   public void solveAndSetTaskSpaceAccelerations(DenseMatrix64F vdotRoot)
   {
      for (MechanismGeometricJacobian jacobian : dMap.keySet())
      {
         DenseMatrix64F d = dMap.get(jacobian);
         DenseMatrix64F jacobianInverse = jacobianInverseMap.get(jacobian);
         vdotTaskSpace.setReshape(d);
         CommonOps.multAdd(-1.0, jacobianInverse, vdotRoot, vdotTaskSpace);
         ScrewTools.setDesiredAccelerations(jacobian.getJointsInOrder(), vdotTaskSpace);
      }
   }

   public void convertInternalSpatialAccelerationToJointSpace(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations,
           MechanismGeometricJacobian jacobian, SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers,
           DenseMatrix64F selectionMatrix)
   {
      jacobian.changeFrame(spatialAcceleration.getExpressedInFrame());
      jacobian.compute();
      DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, null);

      // convectiveTerm
      desiredJointAccelerationCalculator.computeJacobianDerivativeTerm(convectiveTerm);
      convectiveTerm.packMatrix(convectiveTermMatrix, 0);

      // sJ
      sJ.reshape(selectionMatrix.getNumRows(), jacobian.getNumberOfColumns());
      CommonOps.mult(selectionMatrix, jacobian.getJacobianMatrix(), sJ);

      // taskSpaceAcceleration
      spatialAcceleration.packMatrix(taskSpaceAccelerationMatrix, 0);

      // c
      cTaskSpace.reshape(selectionMatrix.getNumRows(), 1);
      cTaskSpace.zero();
      CommonOps.multAdd(1.0, selectionMatrix, taskSpaceAccelerationMatrix, cTaskSpace);
      CommonOps.multAdd(-1.0, selectionMatrix, convectiveTermMatrix, cTaskSpace);

      jacobianSolver.setA(sJ);
      vdotTaskSpace.reshape(cTaskSpace.getNumRows(), 1);
      jacobianSolver.solve(cTaskSpace, vdotTaskSpace);

      int nullity = nullspaceMultipliers.getNumRows();

      if (nullity > 0)
      {
         nullspaceCalculator.setMatrix(sJ, nullity); // TODO: check if this works
         nullspaceCalculator.removeNullspaceComponent(vdotTaskSpace);
         nullspaceCalculator.addNullspaceComponent(vdotTaskSpace, nullspaceMultipliers);
      }

      int index = 0;
      for (InverseDynamicsJoint joint : jacobian.getJointsInOrder())
      {
         DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1); // TODO: garbage
         int degreesOfFreedom = joint.getDegreesOfFreedom();
         CommonOps.extract(vdotTaskSpace, index, index + degreesOfFreedom, 0, 1, jointAcceleration, 0, 0);
         jointSpaceAccelerations.put(joint, jointAcceleration);
         index += degreesOfFreedom;
      }
   }
}
