package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.screwTheory.*;

import java.util.LinkedHashMap;

public class DynamicsMatrixCalculatorHelper
{
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;
   private final JointIndexHandler jointIndexHandler;

   private final LinkedHashMap<InverseDynamicsJoint, int[]> bodyOnlyIndices = new LinkedHashMap<>();

   private final int degreesOfFreedom;
   private final int bodyDoFs;
   private final int floatingBaseDoFs;

   private int rhoSize;

   public DynamicsMatrixCalculatorHelper(GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator, JointIndexHandler jointIndexHandler)
   {
      this.coriolisMatrixCalculator = coriolisMatrixCalculator;
      this.jointIndexHandler = jointIndexHandler;

      degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointIndexHandler.getIndexedJoints());
      OneDoFJoint[] bodyJoints = jointIndexHandler.getIndexedOneDoFJoints();
      bodyDoFs = ScrewTools.computeDegreesOfFreedom(bodyJoints);
      floatingBaseDoFs = degreesOfFreedom - bodyDoFs;

      for (InverseDynamicsJoint joint : bodyJoints)
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

   private final DenseMatrix64F tmpCoriolisMatrix = new DenseMatrix64F(SpatialForceVector.SIZE);
   public void computeCoriolisMatrix(DenseMatrix64F coriolisMatrix)
   {
      InverseDynamicsJoint[] jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();

      for (int jointID = 0; jointID < jointsToOptimizeFor.length; jointID++)
      {
         InverseDynamicsJoint joint = jointsToOptimizeFor[jointID];
         int jointDoFs = joint.getDegreesOfFreedom();
         tmpCoriolisMatrix.reshape(jointDoFs, 1);
         coriolisMatrixCalculator.getJointCoriolisMatrix(joint, tmpCoriolisMatrix);
         int[] jointIndices = jointIndexHandler.getJointIndices(joint);

         for (int i = 0; i < jointDoFs; i++)
         {
            int jointIndex = jointIndices[i];
            CommonOps.extract(tmpCoriolisMatrix, i, i + 1, 0, 1, coriolisMatrix, jointIndex, 0);
         }
      }
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
