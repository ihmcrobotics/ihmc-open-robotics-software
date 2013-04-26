package us.ihmc.commonWalkingControlModules.momentumBasedController;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.MomentumControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.CheckTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * @author twan
 *         Date: 4/25/13
 */
public class OptimizationMomentumControlModule implements MomentumControlModule
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix;    // to make numerical differentiation rewindable

   private final InverseDynamicsJoint rootJoint;    // TODO: make this not be special
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

   private final int nDegreesOfFreedom;
   private int ajIndex = 0;

   private final Map<GeometricJacobian, TaskspaceConstraintData> taskSpaceConstraintMap = new LinkedHashMap<GeometricJacobian, TaskspaceConstraintData>();

   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();
   private final DenseMatrix64F convectiveTermMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F taskSpaceAccelerationMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F JBlock = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F pBlock = new DenseMatrix64F(1, 1);


   private final MomentumOptimizerNative momentumOptimizerNative;

   public OptimizationMomentumControlModule(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame,
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
         columnsForJoints.put(joint, ScrewTools.computeIndicesForJoint(jointsInOrder, joint));
      }

      parentRegistry.addChild(registry);
      reset();

      this.momentumOptimizerNative = new MomentumOptimizerNative();
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

   public void compute(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData momentumRateOfChangeData,
                       LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg)
   {
      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
            controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      CommonOps.mult(centroidalMomentumMatrixDerivative, v, adotV);
   }

   public void resetGroundReactionWrenchFilter()
   {
      // empty for now
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      CheckTools.checkEquals(joint.getDegreesOfFreedom(), jointAcceleration.getNumRows());
      int[] columnsForJoint = this.columnsForJoints.get(joint);
      for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
      {
         AJ.set(ajIndex + i, columnsForJoint[i], 1.0);
      }

      CommonOps.insert(jointAcceleration, bp, ajIndex, 0);
      ajIndex += joint.getDegreesOfFreedom();
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
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
         int[] indicesIntoBlock = ScrewTools.computeIndicesForJoint(baseToEndEffectorJacobian.getJointsInOrder(), joint);
         int[] indicesIntoBigMatrix = columnsForJoints.get(joint);

         for (int i = 0; i < indicesIntoBlock.length; i++)
         {
            int blockIndex = indicesIntoBlock[i];
            int bigMatrixIndex = indicesIntoBigMatrix[i];
            CommonOps.extract(JBlock, 0, JBlock.getNumRows(), blockIndex, blockIndex + 1, AJ, ajIndex, bigMatrixIndex);
         }
      }

      CommonOps.insert(pBlock, bp, ajIndex, 0);

      taskSpaceConstraintMap.put(jacobian, taskspaceConstraintData);

      ajIndex += JBlock.getNumRows();
   }


   public SpatialForceVector getDesiredCentroidalMomentumRate()
   {
      return null;    // TODO: automatically generated code
   }

   public Map<ContactablePlaneBody, Wrench> getExternalWrenches()
   {
      return null;    // TODO: automatically generated code
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
