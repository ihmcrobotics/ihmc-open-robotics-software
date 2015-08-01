package us.ihmc.commonWalkingControlModules.momentumBasedController;

import gnu.trove.list.array.TIntArrayList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;


import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools;


public class MomentumSolver implements MomentumSolverInterface
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DenseMatrix64F vdotRoot;
   private final DenseMatrix64F b;
   private final DenseMatrix64F v;

   private final DenseMatrix64F T;
   private final DenseMatrix64F alpha1;
   private final DenseMatrix64F N;
   private final DenseMatrix64F beta1;

   private final RootJointSolver rootJointSolver;
   private final JointSpaceConstraintResolver jointSpaceConstraintResolver;
   private final TaskSpaceConstraintResolver taskSpaceConstraintResolver;
   private final NullspaceCalculator nullspaceCalculator;

   private final double controlDT;

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix;    // to make numerical differentiation rewindable

   private final InverseDynamicsJoint[] jointsInOrder;
   private final List<InverseDynamicsJoint> jointsInOrderList;
   private final SixDoFJoint rootJoint;


   // LinkedHashMaps so that order of computation is defined
   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();
   private final LinkedHashMap<InverseDynamicsJoint, Boolean> jointAccelerationValidMap = new LinkedHashMap<InverseDynamicsJoint, Boolean>();
   private final LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F> aHats = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
   private final LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
   private final LinkedHashMap<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintData = new LinkedHashMap<GeometricJacobian,
                                                                                                        TaskspaceConstraintData>();


   private final List<InverseDynamicsJoint> unconstrainedJoints = new ArrayList<InverseDynamicsJoint>();

   public MomentumSolver(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
                         LinearSolver<DenseMatrix64F> jacobianSolver, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      this.jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      this.jointsInOrderList = Collections.unmodifiableList(Arrays.asList(jointsInOrder));

      int size = Momentum.SIZE;
      this.nullspaceCalculator = new NullspaceCalculator(size, true);
      this.rootJointSolver = new RootJointSolver();
      this.jointSpaceConstraintResolver = new JointSpaceConstraintResolver();
      this.taskSpaceConstraintResolver = new TaskSpaceConstraintResolver(jointsInOrder, nullspaceCalculator, jacobianSolver);

      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);

      v = new DenseMatrix64F(nDegreesOfFreedom, 1);
      vdotRoot = new DenseMatrix64F(rootJoint.getDegreesOfFreedom(), 1);

      this.controlDT = controlDT;

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());

      b = new DenseMatrix64F(centroidalMomentumMatrixDerivative.getNumRows(), v.getNumCols());

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();
         
         columnsForJoints.put(joint, indices);
         aHats.put(joint, new DenseMatrix64F(size, joint.getDegreesOfFreedom()));
      }

      T = new DenseMatrix64F(size, size);    // will reshape later
      alpha1 = new DenseMatrix64F(size, 1);    // will reshape later
      N = new DenseMatrix64F(size, size);    // will reshape later
      beta1 = new DenseMatrix64F(size, 1);    // will reshape later

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

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
      jointSpaceAccelerations.clear();
      taskspaceConstraintData.clear();
      taskSpaceConstraintResolver.reset();
      unconstrainedJoints.clear();
      unconstrainedJoints.addAll(jointsInOrderList);
      unconstrainedJoints.remove(rootJoint);

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         jointAccelerationValidMap.put(joint, false);
      }
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      checkAndRegisterConstraint(joint, null);
      jointSpaceAccelerations.put(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      InverseDynamicsJoint[] constrainedJoints = jacobian.getJointsInOrder();
      setDesiredSpatialAcceleration(constrainedJoints, jacobian, taskspaceConstraintData);
   }

   public void setDesiredSpatialAcceleration(InverseDynamicsJoint[] constrainedJoints, GeometricJacobian jacobian,
                                             TaskspaceConstraintData taskspaceConstraintData)
   {
      checkNullspaceDimensions(jacobian, taskspaceConstraintData.getNullspaceMultipliers());
      checkSelectionMatrixHasSameNumberOfRowsAsConstrainedJoints(taskspaceConstraintData.getSelectionMatrix(), constrainedJoints);

      for (InverseDynamicsJoint joint : constrainedJoints)
      {
         checkAndRegisterConstraint(joint, jacobian);
      }

      ReferenceFrame baseFrame = taskspaceConstraintData.getSpatialAcceleration().getBaseFrame();
      if (baseFrame == jacobian.getBaseFrame())
      {
         /*
          * TODO: I'd like to get rid of this case and also have it be handled as in the else block below
          * but the results are currently different for some unknown reason, which causes R2StairClimbing to fail.
          */

         taskSpaceConstraintResolver.convertInternalSpatialAccelerationToJointSpace(jointSpaceAccelerations, jacobian,
                 taskspaceConstraintData.getSpatialAcceleration(), taskspaceConstraintData.getNullspaceMultipliers(),
                 taskspaceConstraintData.getSelectionMatrix());
      }
      else
      {
         this.taskspaceConstraintData.put(jacobian, taskspaceConstraintData);
      }
   }

   private void checkSelectionMatrixHasSameNumberOfRowsAsConstrainedJoints(DenseMatrix64F selectionMatrix, InverseDynamicsJoint[] constrainedJoints)
   {
      int numberOfConstrainedJoints = 0;

      for (InverseDynamicsJoint constrainedJoint : constrainedJoints)
      {
         numberOfConstrainedJoints = numberOfConstrainedJoints + constrainedJoint.getDegreesOfFreedom();
      }

      if (selectionMatrix.getNumRows() != numberOfConstrainedJoints)
      {
         throw new RuntimeException("selectionMatrix.getNumRows() != numberOfConstrainedJoints. selectionMatrix.getNumRows() = " + selectionMatrix.getNumRows()
                                    + ", numberOfConstrainedJoints = " + numberOfConstrainedJoints);
      }

   }

   public void compute()
   {
      checkFullyConstrained();

      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      initializeAHats(aHats, centroidalMomentumMatrix.getMatrix());
      initializeB(b, centroidalMomentumMatrixDerivative, jointsInOrder);

      for (GeometricJacobian jacobian : taskspaceConstraintData.keySet())
      {
         TaskspaceConstraintData taskspaceConstraintData = this.taskspaceConstraintData.get(jacobian);
         taskSpaceConstraintResolver.handleTaskSpaceAccelerations(aHats, b, centroidalMomentumMatrix.getMatrix(), jacobian, taskspaceConstraintData);
      }

      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         DenseMatrix64F jointSpaceAcceleration = jointSpaceAccelerations.get(joint);
         jointSpaceConstraintResolver.handleJointSpaceAcceleration(aHats.get(joint), b, centroidalMomentumMatrix.getMatrix(), joint, jointSpaceAcceleration);
      }
   }

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

   public void solve(DenseMatrix64F accelerationSubspace, DenseMatrix64F accelerationMultipliers, DenseMatrix64F momentumSubspace,
                     DenseMatrix64F momentumMultipliers)
   {
      DenseMatrix64F aHatRoot = aHats.get(rootJoint);

      rootJointSolver.solveAndSetRootJointAcceleration(vdotRoot, aHatRoot, b, accelerationSubspace, accelerationMultipliers, momentumSubspace,
            momentumMultipliers, rootJoint, jointAccelerationValidMap);
      jointSpaceConstraintResolver.solveAndSetJointspaceAccelerations(jointSpaceAccelerations, jointAccelerationValidMap);
      taskSpaceConstraintResolver.solveAndSetTaskSpaceAccelerations(jointAccelerationValidMap);

      for (InverseDynamicsJoint joint : jointAccelerationValidMap.keySet())
      {
         if (!jointAccelerationValidMap.get(joint))
            throw new RuntimeException("Joint acceleration for " + joint.getName() + " is invalid at the end of solve.");
      }
   }

   public void getRateOfChangeOfMomentum(SpatialForceVector rateOfChangeOfMomentumToPack)
   {
      rateOfChangeOfMomentumToPack.set(centroidalMomentumMatrix.getReferenceFrame(), rootJointSolver.getRateOfChangeOfMomentum());
   }

   private void initializeB(DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrixDerivative, InverseDynamicsJoint[] jointsInOrder)
   {
      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      CommonOps.mult(centroidalMomentumMatrixDerivative, v, b);
      CommonOps.changeSign(b);
   }

   private void checkFullyConstrained()
   {
      if (unconstrainedJoints.size() > 0)
         throw new RuntimeException("Not fully constrained. Unconstrained joints: " + unconstrainedJoints);
   }

   private void initializeAHats(LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F> aHats, DenseMatrix64F centroidalMomentumMatrix)
   {
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         MatrixTools.extractColumns(centroidalMomentumMatrix, columnsForJoints.get(joint), aHats.get(joint), 0);
      }
   }

   private void checkAndRegisterConstraint(InverseDynamicsJoint joint, GeometricJacobian jacobian)
   {
      if (jacobian != null)
         checkJointIsInJacobian(joint, jacobian);

      if (!unconstrainedJoints.remove(joint))
         throw new RuntimeException("Joint overconstrained: " + joint);
   }

   private static void checkJointIsInJacobian(InverseDynamicsJoint joint, GeometricJacobian jacobian)
   {
      InverseDynamicsJoint[] jointsInJacobian = jacobian.getJointsInOrder();

      boolean foundIt = false;
      for (InverseDynamicsJoint jointInJacobian : jointsInJacobian)
      {
         if (jointInJacobian == joint)
         {
            if (foundIt)
            {
               throw new RuntimeException("Found multiple copies of joint " + joint + " in Jacobian " + jacobian);
            }

            foundIt = true;
         }
      }

      if (!foundIt)
      {
         throw new RuntimeException("Did not find joint " + joint + " in Jacobian " + jacobian);
      }
   }

   private static void checkNullspaceDimensions(GeometricJacobian jacobian, DenseMatrix64F nullspaceMultipliers)
   {
      MathTools.checkIfEqual(nullspaceMultipliers.getNumCols(), 1);
      if (nullspaceMultipliers.getNumRows() > jacobian.getNumberOfColumns())
         throw new RuntimeException("nullspaceMultipliers dimension too large");
   }
}
