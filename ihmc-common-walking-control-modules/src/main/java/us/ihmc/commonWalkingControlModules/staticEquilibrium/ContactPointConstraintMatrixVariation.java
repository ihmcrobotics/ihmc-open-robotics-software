package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.*;
import static us.ihmc.convexOptimization.linearProgram.LinearProgramSolver.computeSensitivity;

public class ContactPointConstraintMatrixVariation
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final WholeBodyContactState wholeBodyContactState;
   private final CenterOfMassStabilityMarginOptimizationModule stabilityMarginOptimizationModule;

   private final DMatrixRMaj equalityConstraintVariation = new DMatrixRMaj(0);
   private final DMatrixRMaj inequalityConstraintVariation = new DMatrixRMaj(0);
   private final DMatrixRMaj solverConstraintVariation = new DMatrixRMaj(0);

   public ContactPointConstraintMatrixVariation(FullHumanoidRobotModel fullRobotModel,
                                                WholeBodyContactState wholeBodyContactState,
                                                CenterOfMassStabilityMarginOptimizationModule stabilityMarginOptimizationModule)
   {
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyContactState = wholeBodyContactState;
      this.stabilityMarginOptimizationModule = stabilityMarginOptimizationModule;
   }

   public DMatrixRMaj compute(int contactPointIndex, Vector3DReadOnly contactPointAdjustment)
   {
      int nominalDecisionVariables = LINEAR_DIMENSIONS * wholeBodyContactState.getNumberOfContactPoints() + CoM_DIMENSIONS;

      equalityConstraintVariation.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, nominalDecisionVariables);
      equalityConstraintVariation.zero();

      int colOffset = 3 * contactPointIndex;

      equalityConstraintVariation.set(3, colOffset + Axis3D.Y.ordinal(), -contactPointAdjustment.getZ());
      equalityConstraintVariation.set(3, colOffset + Axis3D.Z.ordinal(), contactPointAdjustment.getY());
      equalityConstraintVariation.set(4, colOffset + Axis3D.X.ordinal(), contactPointAdjustment.getZ());
      equalityConstraintVariation.set(4, colOffset + Axis3D.Z.ordinal(), -contactPointAdjustment.getX());
      equalityConstraintVariation.set(5, colOffset + Axis3D.X.ordinal(), -contactPointAdjustment.getY());
      equalityConstraintVariation.set(5, colOffset + Axis3D.Y.ordinal(), contactPointAdjustment.getX());

      inequalityConstraintVariation.reshape(2 * equalityConstraintVariation.getNumRows() + wholeBodyContactState.getActuationConstraintMatrix().getNumRows(), nominalDecisionVariables);
      inequalityConstraintVariation.zero();

      MatrixTools.setMatrixBlock(inequalityConstraintVariation, 0, 0, equalityConstraintVariation, 0, 0, equalityConstraintVariation.getNumRows(), equalityConstraintVariation.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(inequalityConstraintVariation, equalityConstraintVariation.getNumRows(), 0, equalityConstraintVariation, 0, 0, equalityConstraintVariation.getNumRows(), equalityConstraintVariation.getNumCols(), -1.0);

      // TODO could optimize by only considering top 12 rows (ignoring joints)
      CommonOps_DDRM.mult(inequalityConstraintVariation, stabilityMarginOptimizationModule.getRhoToForceTransformationMatrix(), solverConstraintVariation);
      return solverConstraintVariation;
   }
}
