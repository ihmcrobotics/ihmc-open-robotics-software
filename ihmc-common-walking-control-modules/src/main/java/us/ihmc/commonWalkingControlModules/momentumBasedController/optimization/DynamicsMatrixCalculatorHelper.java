package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class DynamicsMatrixCalculatorHelper
{
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;

   private final LinkedHashMap<JointBasics, int[]> bodyOnlyIndices = new LinkedHashMap<>();

   private final int degreesOfFreedom;
   private final int bodyDoFs;
   private final int floatingBaseDoFs;

   private int rhoSize;

   public DynamicsMatrixCalculatorHelper(GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator, JointIndexHandler jointIndexHandler)
   {
      this.coriolisMatrixCalculator = coriolisMatrixCalculator;

      degreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(jointIndexHandler.getIndexedJoints());
      OneDoFJointBasics[] bodyJoints = jointIndexHandler.getIndexedOneDoFJoints();
      bodyDoFs = MultiBodySystemTools.computeDegreesOfFreedom(bodyJoints);
      floatingBaseDoFs = degreesOfFreedom - bodyDoFs;

      for (JointBasics joint : bodyJoints)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(bodyJoints, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         bodyOnlyIndices.put(joint, indices);
      }
   }

   public void setRhoSize(int rhoSize)
   {
      this.rhoSize = rhoSize;
   }

   public void computeCoriolisMatrix(DenseMatrix64F coriolisMatrix)
   {
      coriolisMatrix.set(coriolisMatrixCalculator.getJointTauMatrix());
   }

   public void extractFloatingBaseCoriolisMatrix(DenseMatrix64F coriolisMatrixSrc, DenseMatrix64F coriolisMatrixDest)
   {
      CommonOps.extract(coriolisMatrixSrc, 0, floatingBaseDoFs, 0, 1, coriolisMatrixDest, 0, 0);
   }

   public void extractBodyCoriolisMatrix(DenseMatrix64F coriolisMatrixSrc, DenseMatrix64F coriolisMatrixDest)
   {
      CommonOps.extract(coriolisMatrixSrc, floatingBaseDoFs, degreesOfFreedom, 0, 1, coriolisMatrixDest, 0, 0);
   }

   public void extractFloatingBaseContactForceJacobianMatrix(DenseMatrix64F jacobainMatrixSrc, DenseMatrix64F jacobianMatrixDest)
   {
      CommonOps.extract(jacobainMatrixSrc, 0, rhoSize, 0, floatingBaseDoFs, jacobianMatrixDest, 0, 0);
   }

   public void extractBodyContactForceJacobianMatrix(DenseMatrix64F jacobainMatrixSrc, DenseMatrix64F jacobianMatrixDest)
   {
      CommonOps.extract(jacobainMatrixSrc, 0, rhoSize, floatingBaseDoFs, degreesOfFreedom, jacobianMatrixDest, 0, 0);
   }

   public void extractFloatingBaseMassMatrix(DenseMatrix64F massMatrixSrc, DenseMatrix64F massMatrixDest)
   {
      CommonOps.extract(massMatrixSrc, 0, floatingBaseDoFs, 0, degreesOfFreedom, massMatrixDest, 0, 0);
   }

   public void extractBodyMassMatrix(DenseMatrix64F massMatrixSrc, DenseMatrix64F massMatrixDest)
   {
      CommonOps.extract(massMatrixSrc, floatingBaseDoFs, degreesOfFreedom, 0, degreesOfFreedom, massMatrixDest, 0, 0);
   }
}
