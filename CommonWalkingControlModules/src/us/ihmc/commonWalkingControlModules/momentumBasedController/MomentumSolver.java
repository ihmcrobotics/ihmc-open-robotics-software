package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Vector3d;

import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.Pair;
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
import us.ihmc.utilities.screwTheory.Twist;
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

   private final DenseMatrix64F aTaskSpace;
   private final DenseMatrix64F vdotTaskSpace;
   private final DenseMatrix64F cTaskSpace;

   private final DenseMatrix64F b;
   private final DenseMatrix64F adotv;

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final TwistCalculator twistCalculator;
   private final Twist twistOfCurrentWithRespectToNew = new Twist();
   private final Twist twistOfBodyWithRespectToBase = new Twist();

   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final HashMap<MechanismGeometricJacobian, DenseMatrix64F> dMap = new HashMap<MechanismGeometricJacobian, DenseMatrix64F>();
   private final HashMap<MechanismGeometricJacobian, DenseMatrix64F> jacobianInverseMap = new HashMap<MechanismGeometricJacobian, DenseMatrix64F>();
   private final LinearSolver<DenseMatrix64F> jacobianSolver;
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

   private final SpatialForceVector desiredCentroidalMomentumRate;
   private final Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
   private final Map<MechanismGeometricJacobian, Pair<SpatialAccelerationVector, DenseMatrix64F>> spatialAccelerations =
      new HashMap<MechanismGeometricJacobian, Pair<SpatialAccelerationVector, DenseMatrix64F>>();
   private final Vector3d nanVector = new Vector3d(Double.NaN, Double.NaN, Double.NaN);

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

      desiredCentroidalMomentumRate = new SpatialForceVector(centerOfMassFrame);

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
      this.cTaskSpace = new DenseMatrix64F(rowDimension, 1);

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      rows = new int[rowDimension];

      for (int i = 0; i < rowDimension; i++)
      {
         rows[i] = i;
      }

      this.jacobianSolver = jacobianSolver;
      this.nullspaceCalculator = new NullspaceCalculator(rowDimension);

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
      desiredCentroidalMomentumRate.setAngularPart(nanVector);
      desiredCentroidalMomentumRate.setLinearPart(nanVector);
      jointSpaceAccelerations.clear();
      spatialAccelerations.clear();
   }

   public void setDesiredCentroidalMomentumRate(SpatialForceVector desiredCentroidalMomentumRate)
   {
      this.desiredCentroidalMomentumRate.checkAndSet(desiredCentroidalMomentumRate);
   }

   public void setDesiredCentroidalLinearMomentumRate(FrameVector desiredCentroidalLinearMomentumRate)
   {
      this.desiredCentroidalMomentumRate.getExpressedInFrame().checkReferenceFrameMatch(desiredCentroidalLinearMomentumRate.getReferenceFrame());
      this.desiredCentroidalMomentumRate.setLinearPart(desiredCentroidalLinearMomentumRate.getVector());
   }
   
   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      jointSpaceAccelerations.put(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(MechanismGeometricJacobian jacobian,
           Pair<SpatialAccelerationVector, DenseMatrix64F> spatialAccelerationAndNullspaceMultiplier)
   {
      ReferenceFrame baseFrame = spatialAccelerationAndNullspaceMultiplier.first().getBaseFrame();
      boolean isWithRespectToWorld = baseFrame == rootJoint.getPredecessor().getBodyFixedFrame();
      if (isWithRespectToWorld)
      {
         spatialAccelerations.put(jacobian, spatialAccelerationAndNullspaceMultiplier);
      }
      else
      {
         if (baseFrame == jacobian.getBaseFrame())
         {
            convertInternalSpatialAccelerationToJointSpace(jacobian, spatialAccelerationAndNullspaceMultiplier);
         }
         else
         {
            throw new RuntimeException("Case not yet implemented");
         }
      }
   }

   public void solve()
   {
      checkFullyConstrained(jointSpaceAccelerations, spatialAccelerations);
      dMap.clear();
      jacobianInverseMap.clear();

      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      initializeAHatRoot(aHatRoot, centroidalMomentumMatrix.getMatrix());
      initializeB(b, desiredCentroidalMomentumRate, centroidalMomentumMatrixDerivative, jointsInOrder);
      handleJointSpaceAccelerations(b, centroidalMomentumMatrix.getMatrix(), jointSpaceAccelerations);
      handleTaskSpaceAccelerations(aHatRoot, b, dMap, centroidalMomentumMatrix.getMatrix(), spatialAccelerations);

      solveAndSetRootJointAcceleration(vdotRoot, aHatRoot, b, rootJoint);
      solveAndSetJointspaceAccelerations(jointSpaceAccelerations);
      solveAndSetTaskSpaceAccelerations(vdotRoot, dMap, jacobianInverseMap);
   }

   private void initializeB(DenseMatrix64F b, SpatialForceVector desiredCentroidalMomentumRate, DenseMatrix64F centroidalMomentumMatrixDerivative,
                            InverseDynamicsJoint[] jointsInOrder)
   {
      desiredCentroidalMomentumRate.packMatrix(b);
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

   private void handleTaskSpaceAccelerations(DenseMatrix64F aHatRoot, DenseMatrix64F b, HashMap<MechanismGeometricJacobian, DenseMatrix64F> dMap,
           DenseMatrix64F centroidalMomentumMatrix, Map<MechanismGeometricJacobian, Pair<SpatialAccelerationVector, DenseMatrix64F>> taskSpaceAccelerations)
   {
      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      RigidBody rootJointPredecessor = rootJoint.getPredecessor();
      RigidBody rootJointSuccessor = rootJoint.getSuccessor();
      for (MechanismGeometricJacobian jacobian : taskSpaceAccelerations.keySet())
      {
         Pair<SpatialAccelerationVector, DenseMatrix64F> pair = taskSpaceAccelerations.get(jacobian);
         SpatialAccelerationVector taskSpaceAcceleration = pair.first();
         if (taskSpaceAcceleration.getExpressedInFrame() != rootJointFrame)
         {
            twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, rootJointSuccessor, jacobian.getEndEffector());
            twistCalculator.packRelativeTwist(twistOfBodyWithRespectToBase, rootJointPredecessor, jacobian.getEndEffector());
            taskSpaceAcceleration.changeFrame(rootJointSuccessor.getBodyFixedFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
            taskSpaceAcceleration.changeFrameNoRelativeMotion(rootJointFrame);
         }

         // J
         jacobian.changeFrame(rootJointFrame);
         jacobian.compute();

         DenseMatrix64F nullspaceMultiplier = pair.second();
         int nullity = nullspaceMultiplier.getNumCols();

         if (nullity > 0)
         {
            nullspaceCalculator.setMatrix(jacobian.getJacobianMatrix(), nullity);
         }

         // JInverse
         jacobianSolver.setA(jacobian.getJacobianMatrix());
         DenseMatrix64F jacobianInverse = new DenseMatrix64F(jacobian.getNumberOfColumns(), jacobian.getNumberOfColumns());
         jacobianSolver.invert(jacobianInverse);

         if (nullity > 0)
         {
            nullspaceCalculator.removeNullspaceComponent(jacobianInverse);
         }

         jacobianInverseMap.put(jacobian, jacobianInverse);

         // aTaskSpace
         int[] columnIndices = ScrewTools.computeIndicesForJoint(jointsInOrder, jacobian.getJointsInOrder());
         extractMatrixBlock(aTaskSpace, centroidalMomentumMatrix, columnIndices);

         // convectiveTerm
         DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, null);    // TODO: garbage
         desiredJointAccelerationCalculator.computeJacobianDerivativeTerm(convectiveTerm);
         twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, rootJointSuccessor, jacobian.getEndEffector());
         twistCalculator.packRelativeTwist(twistOfBodyWithRespectToBase, jacobian.getBase(), jacobian.getEndEffector());
         convectiveTerm.changeFrame(rootJointSuccessor.getBodyFixedFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
         convectiveTerm.changeFrameNoRelativeMotion(rootJoint.getFrameAfterJoint());
         convectiveTerm.packMatrix(convectiveTermMatrix, 0);

         // cTaskSpace
         convectiveTerm.getBodyFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getBodyFrame());
         convectiveTerm.getExpressedInFrame().checkReferenceFrameMatch(taskSpaceAcceleration.getExpressedInFrame());
         taskSpaceAcceleration.packMatrix(cTaskSpace, 0);
         CommonOps.subEquals(cTaskSpace, convectiveTermMatrix);

         // d
         DenseMatrix64F d = new DenseMatrix64F(jacobianInverse.getNumRows(), cTaskSpace.getNumCols());
         CommonOps.mult(jacobianInverse, cTaskSpace, d);

         if (nullity > 0)
         {
            nullspaceCalculator.addNullspaceComponent(d, nullspaceMultiplier);
         }

         dMap.put(jacobian, d);

         // update aHatRoot
         CommonOps.multAdd(-1.0, aTaskSpace, jacobianInverse, aHatRoot);

         // update b
         CommonOps.multAdd(-1.0, aTaskSpace, d, b);
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

   private void solveAndSetTaskSpaceAccelerations(DenseMatrix64F vdotRoot, HashMap<MechanismGeometricJacobian, DenseMatrix64F> dMap,
           HashMap<MechanismGeometricJacobian, DenseMatrix64F> jacobianInverseMap)
   {
      for (MechanismGeometricJacobian jacobian : dMap.keySet())
      {
         DenseMatrix64F d = dMap.get(jacobian);
         DenseMatrix64F jacobianInverse = jacobianInverseMap.get(jacobian);
         vdotTaskSpace.set(d);
         CommonOps.multAdd(-1.0, jacobianInverse, vdotRoot, vdotTaskSpace);
         ScrewTools.setDesiredAccelerations(jacobian.getJointsInOrder(), vdotTaskSpace);
      }
   }

   private void checkFullyConstrained(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations,
                                      Map<MechanismGeometricJacobian, Pair<SpatialAccelerationVector, DenseMatrix64F>> taskSpaceAccelerations)
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

   private void convertInternalSpatialAccelerationToJointSpace(MechanismGeometricJacobian jacobian, Pair<SpatialAccelerationVector, DenseMatrix64F> pair)
   {
      SpatialAccelerationVector spatialAcceleration = pair.first();
      jacobian.changeFrame(spatialAcceleration.getExpressedInFrame());
      jacobian.compute();
      DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, jacobianSolver);
      desiredJointAccelerationCalculator.compute(spatialAcceleration);

      DenseMatrix64F jointAccelerations = new DenseMatrix64F(jacobian.getNumberOfColumns(), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(jacobian.getJointsInOrder(), jointAccelerations);

      DenseMatrix64F nullspaceMultipliers = pair.second();
      int nullity = nullspaceMultipliers.getNumCols();

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
}
