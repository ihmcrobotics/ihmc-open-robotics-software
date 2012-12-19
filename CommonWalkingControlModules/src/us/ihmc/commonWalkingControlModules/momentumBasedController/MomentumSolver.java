package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.alg.dense.mult.MatrixDimensionException;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;

import us.ihmc.utilities.CheckTools;
import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.NullspaceCalculator;
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

   private final DenseMatrix64F v;

   private final DenseMatrix64F vdotRoot;

   private final DenseMatrix64F aJointSpace;
   private final DenseMatrix64F aJointSpaceVdotJointSpace;

   private final DenseMatrix64F b;
   private final DenseMatrix64F bHat;


   private final DenseMatrix64F aHatRootN;
   private final DenseMatrix64F f;
   private final DenseMatrix64F alpha2Beta2;
   private final DenseMatrix64F alpha2;
   private final DenseMatrix64F beta2;
   private final DenseMatrix64F aVdotRoot1;
   private final DenseMatrix64F hdot;

   private final DenseMatrix64F T;
   private final DenseMatrix64F alpha1;
   private final DenseMatrix64F N;
   private final DenseMatrix64F beta1;
   private final DenseMatrix64F orthogonalCheck;


   private final TaskSpaceConstraintResolver taskSpaceConstraintResolver;
   private final LinearSolver<DenseMatrix64F> jacobianSolver;
   private final LinearSolver<DenseMatrix64F> alpha2Beta2Solver = LinearSolverFactory.linear(SpatialMotionVector.SIZE);
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
   private final int[] rows;

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

      int rowDimension = Momentum.SIZE;
      this.nullspaceCalculator = new NullspaceCalculator(rowDimension);
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

      this.b = new DenseMatrix64F(centroidalMomentumMatrixDerivative.getNumRows(), v.getNumCols());
      this.bHat = new DenseMatrix64F(rowDimension, 1);
      this.aJointSpaceVdotJointSpace = new DenseMatrix64F(Momentum.SIZE, 1);

      this.aHatRoot = new DenseMatrix64F(rowDimension, rootJoint.getDegreesOfFreedom());
      this.aJointSpace = new DenseMatrix64F(rowDimension, rowDimension);    // usually too large, but that's OK because reshape is smart

      this.aHatRootN = new DenseMatrix64F(rowDimension, rowDimension);    // will reshape later
      this.f = new DenseMatrix64F(rowDimension, rowDimension);
      this.alpha2Beta2 = new DenseMatrix64F(rowDimension, 1);
      this.alpha2 = new DenseMatrix64F(rowDimension, 1);    // will reshape later
      this.beta2 = new DenseMatrix64F(rowDimension, 1);    // will reshape later
      this.aVdotRoot1 = new DenseMatrix64F(rowDimension, 1);
      this.hdot = new DenseMatrix64F(rowDimension, 1);

      T = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);    // will reshape later
      alpha1 = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);    // will reshape later
      N = new DenseMatrix64F(SpatialForceVector.SIZE, SpatialForceVector.SIZE);    // will reshape later
      beta1 = new DenseMatrix64F(SpatialForceVector.SIZE, 1);    // will reshape later

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      rows = new int[rowDimension];

      for (int i = 0; i < rowDimension; i++)
      {
         rows[i] = i;
      }

      this.jacobianSolver = jacobianSolver;

      orthogonalCheck = new DenseMatrix64F(rowDimension / 2, rowDimension / 2);    // oversized unless momentumSubspace and accelerationSubspace have 3 columns each

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
         DenseMatrix64F s = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(s);
         this.spatialAccelerations.put(jacobian, spatialAcceleration);
         this.nullspaceMultipliers.put(jacobian, nullspaceMultipliers);
         this.taskSpaceSelectionMatrices.put(jacobian, s);
      }
      else
      {
         if (baseFrame == jacobian.getBaseFrame())
         {
            convertInternalSpatialAccelerationToJointSpace(jacobian, spatialAcceleration, nullspaceMultipliers);
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

      DenseMatrix64F s = new DenseMatrix64F(3, SpatialMotionVector.SIZE);    // TODO: garbage
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
      handleJointSpaceAccelerations(b, centroidalMomentumMatrix.getMatrix(), jointSpaceAccelerations);

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

      solveAndSetAccelerations(T, alpha1, N, beta1);
   }

   public void solve(DenseMatrix64F accelerationSubspace, DenseMatrix64F accelerationMultipliers, DenseMatrix64F momentumSubspace,
                     DenseMatrix64F momentumMultipliers)
   {
      T.setReshape(accelerationSubspace);
      alpha1.setReshape(accelerationMultipliers);
      N.setReshape(momentumSubspace);
      beta1.setReshape(momentumMultipliers);

      solveAndSetAccelerations(T, alpha1, N, beta1);
   }

   public void getRateOfChangeOfMomentum(SpatialForceVector rateOfChangeOfMomentumToPack)
   {
      rateOfChangeOfMomentumToPack.set(centroidalMomentumMatrix.getReferenceFrame(), hdot);
   }

   private void solveAndSetAccelerations(DenseMatrix64F T, DenseMatrix64F alpha1, DenseMatrix64F N, DenseMatrix64F beta1)
   {
      solveAndSetRootJointAcceleration(vdotRoot, aHatRoot, b, T, alpha1, N, beta1, rootJoint);
      solveAndSetJointspaceAccelerations(jointSpaceAccelerations);
      taskSpaceConstraintResolver.solveAndSetTaskSpaceAccelerations(vdotRoot);
   }

   private void initializeB(DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrixDerivative, InverseDynamicsJoint[] jointsInOrder)
   {
      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      CommonOps.mult(centroidalMomentumMatrixDerivative, v, b);
      CommonOps.scale(-1.0, b);
   }

   private void handleJointSpaceAccelerations(DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrix,
           LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         int[] jointSpaceColumnIndices = ScrewTools.computeIndicesForJoint(jointsInOrder, joint);
         aJointSpace.reshape(rows.length, jointSpaceColumnIndices.length);
         MatrixTools.getMatrixBlock(aJointSpace, centroidalMomentumMatrix, rows, jointSpaceColumnIndices);
         DenseMatrix64F vdotJointSpace = jointSpaceAccelerations.get(joint);
         CommonOps.mult(aJointSpace, vdotJointSpace, aJointSpaceVdotJointSpace);
         CommonOps.subEquals(b, aJointSpaceVdotJointSpace);
      }
   }

   private void solveAndSetRootJointAcceleration(DenseMatrix64F vdotRoot, DenseMatrix64F aHatRoot, DenseMatrix64F b, DenseMatrix64F T, DenseMatrix64F alpha1,
           DenseMatrix64F N, DenseMatrix64F beta1, SixDoFJoint rootJoint)
   {
      double epsilonOrthogonal = 1e-10;
      orthogonalCheck.reshape(N.getNumCols(), T.getNumCols());
      CommonOps.multTransA(N, T, orthogonalCheck);
      if (!MatrixFeatures.isConstantVal(orthogonalCheck, 0.0, epsilonOrthogonal))
         throw new RuntimeException("subspaces not orthogonal");

      int momentumSubspaceRank = N.getNumCols();
      int accelerationSubspaceRank = T.getNumCols();

      // aHatRootN
      aHatRootN.reshape(aHatRoot.getNumRows(), N.getNumCols());
      CommonOps.mult(aHatRoot, N, aHatRootN);

      // f
      CommonOps.insert(aHatRootN, f, 0, 0);
      CommonOps.scale(-1.0, T);    // TODO: not so nice
      CommonOps.insert(T, f, 0, momentumSubspaceRank);
      CommonOps.scale(-1.0, T);    // TODO: not so nice

      // vdot1
      checkDimensions(T, alpha1, vdotRoot);
      if (accelerationSubspaceRank == 0)    // handle EJML stupidity
         CommonOps.fill(vdotRoot, 0.0);
      else
         CommonOps.mult(T, alpha1, vdotRoot);

      // hdot1
      checkDimensions(N, beta1, hdot);
      if (momentumSubspaceRank == 0)    // handle EJML stupidity
         CommonOps.fill(hdot, 0.0);
      else
         CommonOps.mult(N, beta1, hdot);

      // aVdot1
      CommonOps.mult(aHatRoot, vdotRoot, aVdotRoot1);

      // bHat
      bHat.set(b);
      CommonOps.addEquals(bHat, hdot);
      CommonOps.subEquals(bHat, aVdotRoot1);

      // alpha2Beta2
      alpha2Beta2Solver.setA(f);
      alpha2Beta2Solver.solve(bHat, alpha2Beta2);

      // alpha2
      alpha2.reshape(momentumSubspaceRank, 1);
      CommonOps.extract(alpha2Beta2, 0, momentumSubspaceRank, 0, 1, alpha2, 0, 0);

      // beta2
      beta2.reshape(accelerationSubspaceRank, 1);
      CommonOps.extract(alpha2Beta2, momentumSubspaceRank, alpha2Beta2.getNumRows(), 0, 1, beta2, 0, 0);

      // vdotRoot
      if (momentumSubspaceRank > 0)    // handle EJML stupidity
         CommonOps.multAdd(N, alpha2, vdotRoot);

      // hdot
      if (accelerationSubspaceRank > 0)    // handle EJML stupidity
         CommonOps.multAdd(T, beta2, hdot);


      DenseMatrix64F check = new DenseMatrix64F(6, 1);
      CommonOps.mult(aHatRoot, vdotRoot, check);
      CommonOps.subEquals(check, hdot);
      CommonOps.subEquals(check, b);

      rootJoint.setDesiredAcceleration(vdotRoot, 0);
   }

   private void checkDimensions(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      if (a.numCols != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if ((a.numRows != c.numRows) || (b.numCols != c.numCols))
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");
   }

   private void solveAndSetJointspaceAccelerations(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         joint.setDesiredAcceleration(jointSpaceAccelerations.get(joint), 0);
      }
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
      MatrixTools.getMatrixBlock(aHatRoot, centroidalMomentumMatrix, rows, columnsForRootJoint);
   }

   private void convertInternalSpatialAccelerationToJointSpace(MechanismGeometricJacobian jacobian, SpatialAccelerationVector spatialAcceleration,
           DenseMatrix64F nullspaceMultipliers)
   {
      jacobian.changeFrame(spatialAcceleration.getExpressedInFrame());
      jacobian.compute();
      DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, jacobianSolver);
      desiredJointAccelerationCalculator.compute(spatialAcceleration);

      DenseMatrix64F jointAccelerations = new DenseMatrix64F(jacobian.getNumberOfColumns(), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(jacobian.getJointsInOrder(), jointAccelerations);

      int nullity = nullspaceMultipliers.getNumRows();

      if (nullity > 0)
      {
         nullspaceCalculator.setMatrix(jacobian.getJacobianMatrix(), nullity);
         nullspaceCalculator.removeNullspaceComponent(jointAccelerations);
         nullspaceCalculator.addNullspaceComponent(jointAccelerations, nullspaceMultipliers);
      }

      int index = 0;
      for (InverseDynamicsJoint joint : jacobian.getJointsInOrder())
      {
         DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         int degreesOfFreedom = joint.getDegreesOfFreedom();
         CommonOps.extract(jointAccelerations, index, index + degreesOfFreedom, 0, 1, jointAcceleration, 0, 0);
         jointSpaceAccelerations.put(joint, jointAcceleration);
         index += degreesOfFreedom;
      }
   }

   private static void checkNullspaceDimensions(MechanismGeometricJacobian jacobian, DenseMatrix64F nullspaceMultipliers)
   {
      CheckTools.checkEquals(nullspaceMultipliers.getNumCols(), 1);
      if (nullspaceMultipliers.getNumRows() > jacobian.getNumberOfColumns())
         throw new RuntimeException("nullspaceMultipliers dimension too large");
   }
}
