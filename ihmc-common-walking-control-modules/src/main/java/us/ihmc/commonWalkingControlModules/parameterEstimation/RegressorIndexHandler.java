package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.robotics.MatrixMissingTools;

public class RegressorIndexHandler
{
   JointReadOnly[] joints;

   RegressorIndexHandler(JointBasics[] joints)
   {
      this.joints = joints;
   }

   public void splitRegressorByJoints(DMatrixRMaj regressor,
                                      boolean[] jointsA, boolean[] jointsB,
                                      DMatrixRMaj regressorAToPack, DMatrixRMaj regressorBToPack)
   {
      splitRegressorByJoints(regressor, jointsA, jointsB, regressorAToPack, regressorBToPack, false);
   }

   /**
    * TODO: Note this assumes that regressorAToPack and regressorBToPack are the correct size, e.g. regressorToPackA should be nDoF x (10 x jointsA.length)
    * @param jointsA
    * @param jointsB
    * @param regressorAToPack
    * @param regressorBToPack
    */
   public void splitRegressorByJoints(DMatrixRMaj regressor,
                                      boolean[] jointsA, boolean[] jointsB,
                                      DMatrixRMaj regressorAToPack, DMatrixRMaj regressorBToPack,
                                      boolean checkDimensions)
   {
      if (checkDimensions)
      {
         // Both A and B should have the same number of rows as the degrees of freedom of the robot
         if (regressorAToPack.getNumRows() != MultiBodySystemTools.computeDegreesOfFreedom(joints) ||
             regressorBToPack.getNumRows() != MultiBodySystemTools.computeDegreesOfFreedom(joints))
            throw new RuntimeException("One of the matrices to pack the regressor partitions has the incorrect number of rows.");

         // A and B should have a number of columns equal to jointsA or jointsB multiplied by 10 (number of inertial parameters per body)
         if (regressorAToPack.getNumCols() != jointsA.length * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY ||
             regressorBToPack.getNumCols() != jointsB.length * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY)
            throw new RuntimeException("One of the matrices to pack the regressor partitions has the incorrect number of columns.");

         // A and B should have a number of columns equal to jointsA or jointsB multiplied by 10 (number of inertial parameters per body)
         if (regressorAToPack.getNumCols() != jointsA.length * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY ||
             regressorBToPack.getNumCols() != jointsB.length * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY)
            throw new RuntimeException("One of the matrices to pack the regressor partitions has the incorrect number of columns.");



      }

      regressorAToPack.zero();
      packRegressorPartition(regressor, jointsA, regressorAToPack);

      regressorBToPack.zero();
      packRegressorPartition(regressor, jointsB, regressorBToPack);
   }

   /**
    * TODO: complete doc
    * @param regressor the regressor to pull from based on the booleans for a given joint
    * @param joints boolean array dictating whether to pull the columns from the regressor corresponding to this joints successor body
    * @param partitionToPack the matrix to pack the regressor columns into, assumed to be of the correct size
    */
   private static void packRegressorPartition(DMatrixRMaj regressor, boolean[] joints, DMatrixRMaj partitionToPack)
   {
      int partitionIndex = 0;
      for (int i = 0; i < joints.length; ++i)
      {
         if (joints[i])
         {
            int regressorStartColumn = i * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
            int regressorBStartColumn = partitionIndex * RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY;
            MatrixMissingTools.setMatrixColumns(partitionToPack, regressorBStartColumn, regressor, regressorStartColumn, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
            partitionIndex += 1;
         }
      }
   }
}
