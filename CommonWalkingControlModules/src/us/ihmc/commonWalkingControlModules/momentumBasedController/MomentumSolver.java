package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumSolver
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DenseMatrix64F jointVelocitiesMatrix;
   private final DenseMatrix64F vdotRoot;
   private final DenseMatrix64F vdotRest;
   private final DenseMatrix64F adotv;
   private final DenseMatrix64F aRoot;
   private final DenseMatrix64F aRest;
   private final DenseMatrix64F aRestvdotRest;
   private final DenseMatrix64F rightHandSide;

   private final double controlDT;

   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix;    // to make numerical differentiation rewindable

   private final YoFrameVector desiredLinearCentroidalMomentumRate;
   private final YoFrameVector desiredAngularCentroidalMomentumRate;

   private final InverseDynamicsJoint[] jointsInOrder;
   private final InverseDynamicsJoint[] restJointsInOrder;
   private final SixDoFJoint rootJoint;

   private final int[] columnsForRootJoint;
   private final int[] columnsForRest;
   private final int[] rows;


   public MomentumSolver(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      this.jointsInOrder = ScrewTools.computeJointsInOrder(elevator);
      this.restJointsInOrder = ScrewTools.computeJointsInOrder(rootJoint.getSuccessor());
      this.columnsForRootJoint = ScrewTools.computeIndicesForJoint(jointsInOrder, new InverseDynamicsJoint[] {rootJoint});
      this.columnsForRest = ScrewTools.computeIndicesForJoint(jointsInOrder, restJointsInOrder);

      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);

      jointVelocitiesMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
      vdotRoot = new DenseMatrix64F(rootJoint.getDegreesOfFreedom(), 1);
      vdotRest = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(restJointsInOrder), 1);

      this.controlDT = controlDT;

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());

      this.rightHandSide = new DenseMatrix64F(centroidalMomentumMatrixDerivative.getNumRows(), jointVelocitiesMatrix.getNumCols());
      this.aRestvdotRest = new DenseMatrix64F(Momentum.SIZE, 1);

      int rowDimension = Momentum.SIZE;
      this.adotv = new DenseMatrix64F(rowDimension, 1);
      this.aRoot = new DenseMatrix64F(rowDimension, rootJoint.getDegreesOfFreedom());
      this.aRest = new DenseMatrix64F(rowDimension, nDegreesOfFreedom - rootJoint.getDegreesOfFreedom());

      this.desiredLinearCentroidalMomentumRate = new YoFrameVector("desiredLinearCentroidalMomentumRate", "", centerOfMassFrame, registry);
      this.desiredAngularCentroidalMomentumRate = new YoFrameVector("desiredAngularCentroidalMomentumRate", "", centerOfMassFrame, registry);

      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      rows = new int[rowDimension];

      for (int i = 0; i < rowDimension; i++)
      {
         rows[i] = i;
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      centroidalMomentumMatrix.compute();
      previousCentroidalMomentumMatrix.set(centroidalMomentumMatrix.getMatrix());
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
   }

   public void solveForRootJointAcceleration(FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate)
   {
      this.desiredAngularCentroidalMomentumRate.set(desiredAngularCentroidalMomentumRate);
      this.desiredLinearCentroidalMomentumRate.set(desiredLinearCentroidalMomentumRate);

      rightHandSide.set(0, 0, desiredAngularCentroidalMomentumRate.getX());
      rightHandSide.set(1, 0, desiredAngularCentroidalMomentumRate.getY());
      rightHandSide.set(2, 0, desiredAngularCentroidalMomentumRate.getZ());
      rightHandSide.set(3, 0, desiredLinearCentroidalMomentumRate.getX());
      rightHandSide.set(4, 0, desiredLinearCentroidalMomentumRate.getY());
      rightHandSide.set(5, 0, desiredLinearCentroidalMomentumRate.getZ());

      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, jointVelocitiesMatrix);
      centroidalMomentumMatrix.compute();
      MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);

      CommonOps.mult(centroidalMomentumMatrixDerivative, jointVelocitiesMatrix, adotv);
      CommonOps.subEquals(rightHandSide, adotv);

      MatrixTools.getMatrixBlock(aRoot, centroidalMomentumMatrix.getMatrix(), rows, columnsForRootJoint);
      MatrixTools.getMatrixBlock(aRest, centroidalMomentumMatrix.getMatrix(), rows, columnsForRest);

      ScrewTools.packDesiredJointAccelerationsMatrix(restJointsInOrder, vdotRest);
      CommonOps.mult(aRest, vdotRest, aRestvdotRest);
      CommonOps.subEquals(rightHandSide, aRestvdotRest);
      CommonOps.solve(aRoot, rightHandSide, vdotRoot);
      rootJoint.setDesiredAcceleration(vdotRoot, 0);
   }
}
