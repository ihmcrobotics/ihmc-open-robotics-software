package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.CheckTools;
import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.NullspaceCalculator;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools;

public class MomentumSolver
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DenseMatrix64F aHatRoot;
   private final DenseMatrix64F vdotRoot;
   private final DenseMatrix64F b;
   private final DenseMatrix64F v;

   private final DenseMatrix64F s;
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

   private final int[] columnsForRootJoint;

   // LinkedHashMaps so that order of computation is defined, to make rewindability checks using reflection easier
   private final LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
   private final LinkedHashMap<MechanismGeometricJacobian, SpatialAccelerationVector> spatialAccelerations = new LinkedHashMap<MechanismGeometricJacobian,
                                                                                                                SpatialAccelerationVector>();
   private final LinkedHashMap<MechanismGeometricJacobian, DenseMatrix64F> nullspaceMultipliers = new LinkedHashMap<MechanismGeometricJacobian,
                                                                                                     DenseMatrix64F>();
   private final LinkedHashMap<MechanismGeometricJacobian, DenseMatrix64F> taskSpaceSelectionMatrices = new LinkedHashMap<MechanismGeometricJacobian,
                                                                                                           DenseMatrix64F>();


   public MomentumSolver(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
                         LinearSolver<DenseMatrix64F> jacobianSolver, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      this.jointsInOrder = ScrewTools.computeJointsInOrder(elevator);
      this.jointsInOrderList = Collections.unmodifiableList(Arrays.asList(jointsInOrder));
      this.columnsForRootJoint = ScrewTools.computeIndicesForJoint(jointsInOrder, new InverseDynamicsJoint[] {rootJoint});

      int size = Momentum.SIZE;
      this.nullspaceCalculator = new NullspaceCalculator(size);
      this.rootJointSolver = new RootJointSolver();
      this.jointSpaceConstraintResolver = new JointSpaceConstraintResolver(jointsInOrder);
      this.taskSpaceConstraintResolver = new TaskSpaceConstraintResolver(rootJoint, jointsInOrder, twistCalculator, nullspaceCalculator, jacobianSolver);

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
      aHatRoot = new DenseMatrix64F(size, rootJoint.getDegreesOfFreedom());

      s = new DenseMatrix64F(size, size);    // will reshape later
      T = new DenseMatrix64F(size, size);    // will reshape later
      alpha1 = new DenseMatrix64F(size, 1);    // will reshape later
      N = new DenseMatrix64F(size, size);    // will reshape later
      beta1 = new DenseMatrix64F(size, 1);    // will reshape later

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      parentRegistry.addChild(registry);
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
      spatialAccelerations.clear();
      nullspaceMultipliers.clear();
      taskSpaceSelectionMatrices.clear();
      taskSpaceConstraintResolver.reset();
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      jointSpaceAccelerations.put(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(MechanismGeometricJacobian jacobian, SpatialAccelerationVector spatialAcceleration,
           DenseMatrix64F nullspaceMultipliers)
   {
      checkNullspaceDimensions(jacobian, nullspaceMultipliers);
      ReferenceFrame baseFrame = spatialAcceleration.getBaseFrame();
      boolean isWithRespectToWorld = baseFrame == rootJoint.getPredecessor().getBodyFixedFrame();
      if (isWithRespectToWorld)
      {
         s.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(s);
         this.spatialAccelerations.put(jacobian, spatialAcceleration);
         this.nullspaceMultipliers.put(jacobian, nullspaceMultipliers);
         this.taskSpaceSelectionMatrices.put(jacobian, s);
      }
      else
      {
         if (baseFrame == jacobian.getBaseFrame())
         {
            taskSpaceConstraintResolver.convertInternalSpatialAccelerationToJointSpace(jointSpaceAccelerations, jacobian, spatialAcceleration,
                    nullspaceMultipliers);
         }
         else
         {
            throw new RuntimeException("Case not yet implemented");
         }
      }
   }

   public void setDesiredAngularAccelerationWithRespectToWorld(MechanismGeometricJacobian jacobian, FrameVector desiredAngularAcceleration,
           DenseMatrix64F nullspaceMultiplier)
   {
      ReferenceFrame elevator = rootJoint.getPredecessor().getBodyFixedFrame();
      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffector().getBodyFixedFrame(), elevator,
                                                         desiredAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      s.reshape(3, SpatialMotionVector.SIZE);
      s.set(0, 0, 1.0);
      s.set(1, 1, 1.0);
      s.set(2, 2, 1.0);

      spatialAccelerations.put(jacobian, spatialAcceleration);
      nullspaceMultipliers.put(jacobian, nullspaceMultiplier);
      taskSpaceSelectionMatrices.put(jacobian, s);
   }

   public void compute()
   {
      checkFullyConstrained(jointSpaceAccelerations, spatialAccelerations);

      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      initializeAHatRoot(aHatRoot, centroidalMomentumMatrix.getMatrix());
      initializeB(b, centroidalMomentumMatrixDerivative, jointsInOrder);

      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         DenseMatrix64F jointSpaceAcceleration = jointSpaceAccelerations.get(joint);
         jointSpaceConstraintResolver.handleJointSpaceAcceleration(b, centroidalMomentumMatrix.getMatrix(), joint, jointSpaceAcceleration);
      }

      for (MechanismGeometricJacobian jacobian : spatialAccelerations.keySet())
      {
         SpatialAccelerationVector taskSpaceAcceleration = spatialAccelerations.get(jacobian);
         DenseMatrix64F nullspaceMultiplier = nullspaceMultipliers.get(jacobian);
         DenseMatrix64F selectionMatrix = taskSpaceSelectionMatrices.get(jacobian);
         taskSpaceConstraintResolver.handleTaskSpaceAccelerations(aHatRoot, b, centroidalMomentumMatrix.getMatrix(), jacobian, taskSpaceAcceleration,
                 nullspaceMultiplier, selectionMatrix);
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
      rootJointSolver.solveAndSetRootJointAcceleration(vdotRoot, aHatRoot, b, accelerationSubspace, accelerationMultipliers, momentumSubspace,
              momentumMultipliers, rootJoint);
      jointSpaceConstraintResolver.solveAndSetJointspaceAccelerations(jointSpaceAccelerations);
      taskSpaceConstraintResolver.solveAndSetTaskSpaceAccelerations(vdotRoot);
   }

   public void getRateOfChangeOfMomentum(SpatialForceVector rateOfChangeOfMomentumToPack)
   {
      rateOfChangeOfMomentumToPack.set(centroidalMomentumMatrix.getReferenceFrame(), rootJointSolver.getRateOfChangeOfMomentum());
   }

   private void initializeB(DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrixDerivative, InverseDynamicsJoint[] jointsInOrder)
   {
      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      CommonOps.mult(centroidalMomentumMatrixDerivative, v, b);
      CommonOps.scale(-1.0, b);
   }

   private void checkFullyConstrained(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations,
                                      LinkedHashMap<MechanismGeometricJacobian, SpatialAccelerationVector> spatialAccelerations)
   {
      int totalJointsWithPossibleDuplication = 0;
      Set<InverseDynamicsJoint> allJoints = new HashSet<InverseDynamicsJoint>(jointsInOrder.length);
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         allJoints.add(joint);
         totalJointsWithPossibleDuplication++;
      }

      for (MechanismGeometricJacobian jacobian : spatialAccelerations.keySet())
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
      MatrixTools.extractColumns(centroidalMomentumMatrix, aHatRoot, columnsForRootJoint);
   }

   private static void checkNullspaceDimensions(MechanismGeometricJacobian jacobian, DenseMatrix64F nullspaceMultipliers)
   {
      CheckTools.checkEquals(nullspaceMultipliers.getNumCols(), 1);
      if (nullspaceMultipliers.getNumRows() > jacobian.getNumberOfColumns())
         throw new RuntimeException("nullspaceMultipliers dimension too large");
   }
}
