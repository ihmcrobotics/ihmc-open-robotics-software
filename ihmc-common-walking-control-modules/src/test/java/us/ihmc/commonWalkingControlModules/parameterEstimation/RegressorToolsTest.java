package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.*;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.*;

public class RegressorToolsTest
{
   private static final double EPSILON = 1e-9;
   private static final int ITERATIONS = 1000;
   
   private static final int PARAMETERS_PER_RIGID_BODY = 10;

   @Test
   public void testPartitionRegressor()
   {
      Random random = new Random(345357L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int numberOfJoints = random.nextInt(30) + 1;  // to avoid zero
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         Set<SpatialInertiaBasisOption>[] basisSets = new Set[numberOfJoints];
         for (int j = 0; j < numberOfJoints; ++j)
         {
            basisSets[j] = new HashSet<>();

            // Randomly add basis options to the set
            for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
            {
               if (random.nextBoolean())
                  basisSets[j].add(option);
            }
         }

         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

         JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
         regressorCalculator.setGravitationalAcceleration(-9.81);
         regressorCalculator.compute();
         DMatrixRMaj regressor = regressorCalculator.getJointTorqueRegressorMatrix();

         // Use the tools to appropriately size the partition matrices, based on the number of elements in the list of basis sets
         DMatrixRMaj[] partitionMatrices = RegressorTools.sizePartitionMatrices(regressor, basisSets);

         RegressorTools.partitionRegressor(regressor, basisSets, partitionMatrices[0], partitionMatrices[1]);  // actual
         DMatrixRMaj[] expectedPartitionMatrices = computeRegressorPartitions(regressor, basisSets);  // expected (alternative implementation)

         for (int k = 0; k < partitionMatrices.length; ++k)
            assertArrayEquals(expectedPartitionMatrices[k].getData(), partitionMatrices[k].getData(), EPSILON);
      }
   }

   @Test
   public void testPartitionRegressorInputChecks()
   {
      Random random = new Random(5867L);
      int numberOfJoints = 2;
      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

      Set<SpatialInertiaBasisOption>[] basisSets = new Set[numberOfJoints];
      for (int j = 0; j < numberOfJoints; ++j)
      {
         basisSets[j] = new HashSet<>();

         // Randomly add basis options to the set
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            if (random.nextBoolean())
               basisSets[j].add(option);
         }
      }

      for (JointStateType stateToRandomize : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

      JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
      regressorCalculator.setGravitationalAcceleration(-9.81);
      regressorCalculator.compute();
      DMatrixRMaj regressor = regressorCalculator.getJointTorqueRegressorMatrix();

      int nDoFs = system.getNumberOfDoFs();
      int[] partitionSizes = RegressorTools.getPartitionSizes(basisSets);

      // Corrupt the partition matrices to have the incorrect number of rows -- should throw exception
      // partitionMatrices[0] has one more row than needed, partitionMatrices[1] has one less row than needed
      DMatrixRMaj[] partitionMatrices = new DMatrixRMaj[] {new DMatrixRMaj(nDoFs + 1, partitionSizes[0]), new DMatrixRMaj(nDoFs - 1, partitionSizes[1])};
      try
      {
         boolean checkInputs = true;
         RegressorTools.partitionRegressor(regressor, basisSets, partitionMatrices[0], partitionMatrices[1], checkInputs);
         fail("Should have thrown exception");
      }
      catch (RuntimeException e)
      {
         // good
         System.out.println(e.getMessage());
      }

      // Corrupt the partition matrices to have the incorrect number of columns -- should throw an exception
      // The sum of the number of columns of both partition matrices should equal the number of columns of the regressor, add one column to both partition
      // matrices to make this fail
      partitionMatrices[0] = new DMatrixRMaj(nDoFs, partitionSizes[0] + 1);
      partitionMatrices[1] = new DMatrixRMaj(nDoFs, partitionSizes[1] + 1);
      try
      {
         boolean checkInputs = true;
         RegressorTools.partitionRegressor(regressor, basisSets, partitionMatrices[0], partitionMatrices[1], checkInputs);
         fail("Should have thrown exception");
      }
      catch (RuntimeException e)
      {
         // good
         System.out.println(e.getMessage());
      }

      // Corrupt the basis sets such that the total number of basis entries no longer equals the number of columns of the regressor -- should throw exception
      // Empty the first basis set
      basisSets[0].clear();
      // Both partition matrices are correct
      partitionMatrices[0] = new DMatrixRMaj(nDoFs, partitionSizes[0]);
      partitionMatrices[1] = new DMatrixRMaj(nDoFs, partitionSizes[1]);
      try
      {
         boolean checkInputs = true;
         RegressorTools.partitionRegressor(regressor, basisSets, partitionMatrices[0], partitionMatrices[1], checkInputs);
         fail("Should have thrown exception");
      }
      catch (RuntimeException e)
      {
         // good
         System.out.println(e.getMessage());
      }
   }

   private DMatrixRMaj[] computeRegressorPartitions(DMatrixRMaj regressor, Set<SpatialInertiaBasisOption>[] basisSets)
   {
      List<Integer> collectionIndices = new ArrayList<>();
      List<Integer> collectionComplementIndices = new ArrayList<>();

      for (int i = 0; i < basisSets.length; ++i)
      {
         for (int j = 0; j < PARAMETERS_PER_RIGID_BODY; ++j)
         {
            if (basisSets[i].contains(SpatialInertiaBasisOption.values[j]))
               collectionIndices.add(i * PARAMETERS_PER_RIGID_BODY + j);
            else
               collectionComplementIndices.add(i * PARAMETERS_PER_RIGID_BODY + j);
         }
      }

      DMatrixRMaj collectionPartition = new DMatrixRMaj(regressor.getNumRows(), collectionIndices.size());
      DMatrixRMaj collectionComplementPartition = new DMatrixRMaj(regressor.getNumRows(), collectionComplementIndices.size());

      for (int k = 0; k < collectionIndices.size(); ++k)
         MatrixMissingTools.setMatrixColumn(collectionPartition, k, regressor, collectionIndices.get(k));

      for (int l = 0; l < collectionComplementIndices.size(); ++l)
         MatrixMissingTools.setMatrixColumn(collectionComplementPartition, l, regressor, collectionComplementIndices.get(l));

      return new DMatrixRMaj[] {collectionPartition, collectionComplementPartition};
   }
}