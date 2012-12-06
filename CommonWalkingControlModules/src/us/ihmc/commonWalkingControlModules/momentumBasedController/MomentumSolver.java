package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.DesiredJointAccelerationCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumSolver
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DenseMatrix64F aHatRoot;

   private final DenseMatrix64F v;

   private final DenseMatrix64F vdotRoot;

   private final DenseMatrix64F aJointSpace;
   private final DenseMatrix64F aJointSpaceVdotJointSpace;

   private final DenseMatrix64F aTaskSpace;
   private final DenseMatrix64F vdotTaskSpace;
   private final DenseMatrix64F aHatTaskSpace;
   private final DenseMatrix64F aHatcTaskSpace;

   private final DenseMatrix64F b;
   private final DenseMatrix64F adotv;

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final TwistCalculator twistCalculator;
   private final Twist twistOfCurrentWithRespectToNew = new Twist();
   private final Twist twistOfBodyWithRespectToBase = new Twist();

   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final LinearSolver<DenseMatrix64F> jacobianSolver;
   private final DenseMatrix64F jacobianInverse = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final HashMap<MechanismGeometricJacobian, DenseMatrix64F> cTaskSpaceMap = new HashMap<MechanismGeometricJacobian, DenseMatrix64F>();

   private final double controlDT;

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix;    // to make numerical differentiation rewindable

   private final YoFrameVector desiredLinearCentroidalMomentumRate;
   private final YoFrameVector desiredAngularCentroidalMomentumRate;

   private final InverseDynamicsJoint[] jointsInOrder;
   private final List<InverseDynamicsJoint> jointsInOrderList;
   private final SixDoFJoint rootJoint;

   private final int[] columnsForRootJoint;
   private final int[] rows;


   public MomentumSolver(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
                         LinearSolver<DenseMatrix64F> jacobianSolver, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      this.twistCalculator = twistCalculator;
      this.jointsInOrder = ScrewTools.computeJointsInOrder(elevator);
      this.jointsInOrderList = Collections.unmodifiableList(Arrays.asList(jointsInOrder));
      this.columnsForRootJoint = ScrewTools.computeIndicesForJoint(jointsInOrder, new InverseDynamicsJoint[] {rootJoint});

      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      int rowDimension = Momentum.SIZE;

      v = new DenseMatrix64F(nDegreesOfFreedom, 1);
      vdotRoot = new DenseMatrix64F(rootJoint.getDegreesOfFreedom(), 1);

      this.controlDT = controlDT;

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());

      this.b = new DenseMatrix64F(centroidalMomentumMatrixDerivative.getNumRows(), v.getNumCols());
      this.aJointSpaceVdotJointSpace = new DenseMatrix64F(Momentum.SIZE, 1);

      this.adotv = new DenseMatrix64F(rowDimension, 1);
      this.aHatRoot = new DenseMatrix64F(rowDimension, rootJoint.getDegreesOfFreedom());
      this.aJointSpace = new DenseMatrix64F(rowDimension, rowDimension);    // usually too large, but that's OK because reshape is smart

      this.aTaskSpace = new DenseMatrix64F(rowDimension, rowDimension);
      this.vdotTaskSpace = new DenseMatrix64F(rowDimension, 1);
      this.aHatTaskSpace = new DenseMatrix64F(rowDimension, rowDimension);
      this.aHatcTaskSpace = new DenseMatrix64F(rowDimension, 1);

      this.desiredLinearCentroidalMomentumRate = new YoFrameVector("desiredLinearCentroidalMomentumRate", "", centerOfMassFrame, registry);
      this.desiredAngularCentroidalMomentumRate = new YoFrameVector("desiredAngularCentroidalMomentumRate", "", centerOfMassFrame, registry);

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      rows = new int[rowDimension];

      for (int i = 0; i < rowDimension; i++)
      {
         rows[i] = i;
      }

      this.jacobianSolver = jacobianSolver;

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      centroidalMomentumMatrix.compute();
      previousCentroidalMomentumMatrix.set(centroidalMomentumMatrix.getMatrix());
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
   }

   public void solve(FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate,
                     Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations,
                     Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations)
   {
      checkFullyConstrained(jointSpaceAccelerations, taskSpaceAccelerations);

      this.desiredAngularCentroidalMomentumRate.set(desiredAngularCentroidalMomentumRate);
      this.desiredLinearCentroidalMomentumRate.set(desiredLinearCentroidalMomentumRate);

      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      initializeAHatRoot(aHatRoot, centroidalMomentumMatrix.getMatrix());
      initializeB(b, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, centroidalMomentumMatrixDerivative, jointsInOrder);
      handleJointSpaceAccelerations(b, centroidalMomentumMatrix.getMatrix(), jointSpaceAccelerations);
      handleTaskSpaceAccelerations(aHatRoot, b, cTaskSpaceMap, centroidalMomentumMatrix.getMatrix(), taskSpaceAccelerations);

      solveAndSetRootJointAcceleration(vdotRoot, aHatRoot, b, rootJoint);
      solveAndSetJointspaceAccelerations(jointSpaceAccelerations);
      solveAndSetTaskSpaceAccelerations(vdotRoot, cTaskSpaceMap, taskSpaceAccelerations);
   }

   private void initializeB(DenseMatrix64F b, FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate,
                            DenseMatrix64F centroidalMomentumMatrixDerivative, InverseDynamicsJoint[] jointsInOrder)
   {
      b.set(0, 0, desiredAngularCentroidalMomentumRate.getX());
      b.set(1, 0, desiredAngularCentroidalMomentumRate.getY());
      b.set(2, 0, desiredAngularCentroidalMomentumRate.getZ());
      b.set(3, 0, desiredLinearCentroidalMomentumRate.getX());
      b.set(4, 0, desiredLinearCentroidalMomentumRate.getY());
      b.set(5, 0, desiredLinearCentroidalMomentumRate.getZ());

      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      CommonOps.mult(centroidalMomentumMatrixDerivative, v, adotv);
      CommonOps.subEquals(b, adotv);
   }

   private void handleJointSpaceAccelerations(DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrix,
           Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         int[] jointSpaceColumnIndices = ScrewTools.computeIndicesForJoint(jointsInOrder, joint);
         extractMatrixBlock(aJointSpace, centroidalMomentumMatrix, jointSpaceColumnIndices);
         DenseMatrix64F vdotJointSpace = jointSpaceAccelerations.get(joint);
         CommonOps.mult(this.aJointSpace, vdotJointSpace, aJointSpaceVdotJointSpace);
         CommonOps.subEquals(b, aJointSpaceVdotJointSpace);
      }
   }

   private void handleTaskSpaceAccelerations(DenseMatrix64F aHatRoot, DenseMatrix64F b, HashMap<MechanismGeometricJacobian, DenseMatrix64F> cTaskSpaceMap,
           DenseMatrix64F centroidalMomentumMatrix, Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations)
   {
      cTaskSpaceMap.clear();
      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      RigidBody rootJointPredecessor = rootJoint.getPredecessor();
      RigidBody rootJointSuccessor = rootJoint.getSuccessor();
      for (MechanismGeometricJacobian jacobian : taskSpaceAccelerations.keySet())
      {
         SpatialAccelerationVector taskSpaceAcceleration = taskSpaceAccelerations.get(jacobian);
         if (taskSpaceAcceleration.getExpressedInFrame() != rootJointFrame)
         {
            twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, rootJointSuccessor, jacobian.getEndEffector());
            twistCalculator.packRelativeTwist(twistOfBodyWithRespectToBase, rootJointPredecessor, jacobian.getEndEffector());
            taskSpaceAcceleration.changeFrame(rootJointSuccessor.getBodyFixedFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
            taskSpaceAcceleration.changeFrameNoRelativeMotion(rootJointFrame);
         }

         // JInverse
         jacobian.compute();
         jacobianSolver.setA(jacobian.getJacobianMatrix());
         jacobianSolver.invert(jacobianInverse);

         // aTaskSpace
         int[] indices = ScrewTools.computeIndicesForJoint(jointsInOrder, jacobian.getJointsInOrder());
         extractMatrixBlock(aTaskSpace, centroidalMomentumMatrix, indices);

         // aHatTaskSpace
         CommonOps.mult(aTaskSpace, jacobianInverse, aHatTaskSpace);

         // convectiveTerm
         DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, null);    // TODO: garbage
         desiredJointAccelerationCalculator.computeJacobianDerivativeTerm(convectiveTerm);
         twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, rootJointSuccessor, jacobian.getEndEffector());
         convectiveTerm.changeFrame(rootJointSuccessor.getBodyFixedFrame(), twistOfCurrentWithRespectToNew, twistOfCurrentWithRespectToNew);
         convectiveTerm.changeFrameNoRelativeMotion(rootJoint.getFrameAfterJoint());
         convectiveTerm.packMatrix(convectiveTermMatrix, 0);

         // cTaskSpace
         convectiveTerm.getBodyFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getBodyFrame());
         convectiveTerm.getExpressedInFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getExpressedInFrame());
         DenseMatrix64F cTaskSpace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);    // TODO: garbage
         taskSpaceAcceleration.packMatrix(cTaskSpace, 0);
         CommonOps.subEquals(cTaskSpace, convectiveTermMatrix);
         cTaskSpaceMap.put(jacobian, cTaskSpace);

         // aHatCTaskSpace
         CommonOps.mult(aHatTaskSpace, cTaskSpace, aHatcTaskSpace);

         // update aHatRoot
         CommonOps.subEquals(aHatRoot, aHatTaskSpace);

         // update b
         CommonOps.subEquals(b, aHatcTaskSpace);
      }
   }

   private void solveAndSetRootJointAcceleration(DenseMatrix64F vdotRoot, DenseMatrix64F aHatRoot, DenseMatrix64F b, SixDoFJoint rootJoint)
   {
      CommonOps.solve(aHatRoot, b, vdotRoot);
      rootJoint.setDesiredAcceleration(vdotRoot, 0);
   }

   private void solveAndSetJointspaceAccelerations(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         joint.setDesiredAcceleration(jointSpaceAccelerations.get(joint), 0);
      }
   }

   private void solveAndSetTaskSpaceAccelerations(DenseMatrix64F vdotRoot, HashMap<MechanismGeometricJacobian, DenseMatrix64F> cTaskSpaceMap,
           Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations)
   {
      // TODO: nullspace
      for (MechanismGeometricJacobian jacobian : taskSpaceAccelerations.keySet())
      {
         jacobianSolver.setA(jacobian.getJacobianMatrix());
         DenseMatrix64F cTaskSpace = cTaskSpaceMap.get(jacobian);
         CommonOps.subEquals(cTaskSpace, vdotRoot);
         jacobianSolver.solve(cTaskSpace, vdotTaskSpace);
         ScrewTools.setDesiredAccelerations(jacobian.getJointsInOrder(), vdotTaskSpace);
      }
   }

   private void checkFullyConstrained(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations,
                                      Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations)
   {
      int totalJointsWithPossibleDuplication = 0;
      Set<InverseDynamicsJoint> allJoints = new HashSet<InverseDynamicsJoint>(jointsInOrder.length);
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         allJoints.add(joint);
         totalJointsWithPossibleDuplication++;
      }

      for (MechanismGeometricJacobian jacobian : taskSpaceAccelerations.keySet())
      {
         for (InverseDynamicsJoint taskSpaceJoint : jacobian.getJointsInOrder())
         {
            allJoints.add(taskSpaceJoint);
            totalJointsWithPossibleDuplication++;
         }
      }

      allJoints.add(rootJoint);
      totalJointsWithPossibleDuplication++;

      if (totalJointsWithPossibleDuplication != allJoints.size())
         throw new RuntimeException("There are multiply constrained joints");

      if (!allJoints.containsAll(jointsInOrderList))
         throw new RuntimeException("Not correctly constrained");
   }

   private void initializeAHatRoot(DenseMatrix64F aHatRoot, DenseMatrix64F centroidalMomentumMatrix)
   {
      MatrixTools.getMatrixBlock(aHatRoot, centroidalMomentumMatrix, rows, columnsForRootJoint);
   }

   private void extractMatrixBlock(DenseMatrix64F matrixToPack, DenseMatrix64F centroidalMomentumMatrix, int[] jointSpaceIndices)
   {
      matrixToPack.reshape(rows.length, jointSpaceIndices.length);
      MatrixTools.getMatrixBlock(matrixToPack, centroidalMomentumMatrix, rows, jointSpaceIndices);
   }
}
