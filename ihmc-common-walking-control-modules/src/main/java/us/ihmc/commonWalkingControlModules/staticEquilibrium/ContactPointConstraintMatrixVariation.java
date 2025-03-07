package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.CoM_DIMENSIONS;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.NUM_DYNAMICS_CONSTRAINTS;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginOptimizationModule.*;

public class ContactPointConstraintMatrixVariation
{
   private final StabilityMarginOptimizationModule stabilityMarginOptimizationModule;

   private final DMatrixRMaj equalityConstraintVariation = new DMatrixRMaj(0);
   private final DMatrixRMaj inequalityConstraintVariation = new DMatrixRMaj(0);
   private final DMatrixRMaj solverConstraintVariation = new DMatrixRMaj(0);

   public ContactPointConstraintMatrixVariation(StabilityMarginOptimizationModule stabilityMarginOptimizationModule)
   {
      this.stabilityMarginOptimizationModule = stabilityMarginOptimizationModule;
   }

   public DMatrixRMaj compute(int contactPointIndex, Vector3DReadOnly contactPointAdjustment)
   {
      int numEqualityDynamicsConstraints = stabilityMarginOptimizationModule.getNumDynamicsConstraints();
      DMatrixRMaj solverToNominalTransformation = stabilityMarginOptimizationModule.getSolverToNominalTransformation();
      DMatrixRMaj nominalConstraintMatrix = stabilityMarginOptimizationModule.getNominalConstraintMatrix();

      /* Always is point mass  */
      int numAngularDynamicsConstraints = 3;
      /* Is either XYZ or Z  */
      int numLinearDynamicsConstraints = stabilityMarginOptimizationModule.getNumDynamicsConstraints() - numAngularDynamicsConstraints;

      equalityConstraintVariation.reshape(numEqualityDynamicsConstraints, nominalConstraintMatrix.getNumCols());
      equalityConstraintVariation.zero();

      int colOffset = LINEAR_DIMENSIONS * contactPointIndex;

      equalityConstraintVariation.set(numLinearDynamicsConstraints + Axis3D.X.ordinal(), colOffset + Axis3D.Y.ordinal(), -contactPointAdjustment.getZ());
      equalityConstraintVariation.set(numLinearDynamicsConstraints + Axis3D.X.ordinal(), colOffset + Axis3D.Z.ordinal(), contactPointAdjustment.getY());
      equalityConstraintVariation.set(numLinearDynamicsConstraints + Axis3D.Y.ordinal(), colOffset + Axis3D.X.ordinal(), contactPointAdjustment.getZ());
      equalityConstraintVariation.set(numLinearDynamicsConstraints + Axis3D.Y.ordinal(), colOffset + Axis3D.Z.ordinal(), -contactPointAdjustment.getX());
      equalityConstraintVariation.set(numLinearDynamicsConstraints + Axis3D.Z.ordinal(), colOffset + Axis3D.X.ordinal(), -contactPointAdjustment.getY());
      equalityConstraintVariation.set(numLinearDynamicsConstraints + Axis3D.Z.ordinal(), colOffset + Axis3D.Y.ordinal(), contactPointAdjustment.getX());

      inequalityConstraintVariation.reshape(nominalConstraintMatrix.getNumRows(), nominalConstraintMatrix.getNumCols());
      inequalityConstraintVariation.zero();

      MatrixTools.setMatrixBlock(inequalityConstraintVariation, 0, 0, equalityConstraintVariation, 0, 0, equalityConstraintVariation.getNumRows(), equalityConstraintVariation.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(inequalityConstraintVariation, equalityConstraintVariation.getNumRows(), 0, equalityConstraintVariation, 0, 0, equalityConstraintVariation.getNumRows(), equalityConstraintVariation.getNumCols(), -1.0);

      CommonOps_DDRM.mult(inequalityConstraintVariation, solverToNominalTransformation, solverConstraintVariation);
      return solverConstraintVariation;
   }
}
