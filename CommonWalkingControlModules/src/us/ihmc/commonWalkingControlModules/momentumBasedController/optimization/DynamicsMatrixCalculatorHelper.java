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

   public DynamicsMatrixCalculatorHelper(GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator, JointIndexHandler jointIndexHandler)
   {
      this.coriolisMatrixCalculator = coriolisMatrixCalculator;
      this.jointIndexHandler = jointIndexHandler;

      OneDoFJoint[] bodyJoints = jointIndexHandler.getIndexedOneDoFJoints();
      for (InverseDynamicsJoint joint : bodyJoints)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(bodyJoints, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         bodyOnlyIndices.put(joint, indices);
      }
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

   public void extractFloatingBaseCoriolisMatrix(FloatingInverseDynamicsJoint joint, DenseMatrix64F coriolisMatrixSrc, DenseMatrix64F coriolisMatrixDest)
   {
      int[] jointIndices = jointIndexHandler.getJointIndices(joint);

      for (int jointNumber = 0; jointNumber < jointIndices.length; jointNumber++)
      {
         int jointIndex = jointIndices[jointNumber];

         CommonOps.extract(coriolisMatrixSrc, jointIndex, jointIndex + 1, 0, 1, coriolisMatrixDest, jointNumber, 0);
      }
   }

   public void extractBodyCoriolisMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F coriolisMatrixSrc, DenseMatrix64F coriolisMatrixDest)
   {
      for (int i = 0; i < joints.length; i++)
         extractBodyCoriolisMatrix(joints[i], coriolisMatrixSrc, coriolisMatrixDest);
   }

   public void extractBodyCoriolisMatrix(InverseDynamicsJoint joint, DenseMatrix64F coriolisMatrixSrc, DenseMatrix64F coriolisMatrixDest)
   {
      int[] fullJointIndices = jointIndexHandler.getJointIndices(joint);
      int[] bodyJointIndices = bodyOnlyIndices.get(joint);

      for (int jointNumber = 0; jointNumber < fullJointIndices.length; jointNumber++)
      {
         int fullJointIndex = fullJointIndices[jointNumber];
         int bodyJointIndex = bodyJointIndices[jointNumber];

         CommonOps.extract(coriolisMatrixSrc, fullJointIndex, fullJointIndex + 1, 0, 1, coriolisMatrixDest, bodyJointIndex, 0);
      }
   }

   // // TODO: 1/2/17  
   public void extractFloatingBaseMassMatrix(FloatingInverseDynamicsJoint joint, DenseMatrix64F massMatrixSrc, DenseMatrix64F massMatrixDest)
   {
   }

   // // TODO: 1/2/17
   public void extractBodyMassMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F massMatrixSrc, DenseMatrix64F massMatrixDest)
   {
      for (int i = 0; i < joints.length; i++)
         extractBodyCoriolisMatrix(joints[i], massMatrixSrc, massMatrixDest);
   }

   // // TODO: 1/2/17
   public void extractBodyMassMatrix(InverseDynamicsJoint joint, DenseMatrix64F massMatrixSrc, DenseMatrix64F massMatrixDest)
   {
      int[] fullJointIndices = jointIndexHandler.getJointIndices(joint);
      int[] bodyJointIndices = bodyOnlyIndices.get(joint);

      for (int jointNumber = 0; jointNumber < fullJointIndices.length; jointNumber++)
      {
         int fullJointIndex = fullJointIndices[jointNumber];
         int bodyJointIndex = bodyJointIndices[jointNumber];

         CommonOps.extract(massMatrixSrc, fullJointIndex, fullJointIndex + 1, 0, 1, massMatrixDest, bodyJointIndex, 0);
      }
   }
}
