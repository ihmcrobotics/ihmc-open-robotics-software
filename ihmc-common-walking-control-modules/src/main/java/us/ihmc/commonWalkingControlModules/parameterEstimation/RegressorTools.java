package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.Set;

import static us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;

public class RegressorTools
{
   private static final int PARAMETERS_PER_RIGID_BODY = 10;

   /**
    * Partition a regressor matrix into two matrices, one matrix containing the regressor columns corresponding to the spatial inertia bases in the list of
    * {@code SpatialInertiaBasisOption} sets, and the other matrix containing the remaining regressor columns.
    * <p>
    * NOTE: This method assumes that the two partition matrices, {@code collectionPartitionToPack} and {@code collectionComplementPartitionToPack}, have been
    * correctly sized. See {@link #sizePartitionMatrices(DMatrixRMaj, Set[])}.
    * </p>
    *
    * @param regressor                           the regressor matrix to be partitioned. Not modified.
    * @param basisSets                           the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the regressor. Not modified.
    * @param collectionPartitionToPack           the matrix in which the regressor columns corresponding to the spatial inertia bases in the list of basis sets
    *                                            are stored. Modified.
    * @param collectionComplementPartitionToPack the matrix in which the remaining regressor columns are stored. Modified.
    */
   public static void partitionRegressor(DMatrixRMaj regressor,
                                         Set<SpatialInertiaBasisOption>[] basisSets,
                                         DMatrixRMaj collectionPartitionToPack,
                                         DMatrixRMaj collectionComplementPartitionToPack)
   {
      partitionRegressor(regressor, basisSets, collectionPartitionToPack, collectionComplementPartitionToPack, false);
   }

   /**
    * Partition a regressor matrix into two matrices, one matrix containing the regressor columns corresponding to the spatial inertia bases in the list of
    * {@code SpatialInertiaBasisOption} sets, and the other matrix containing the remaining regressor columns.
    * <p>
    * NOTE: This method will perform checks to ensure that the two partition matrices, {@code collectionPartitionToPack} and
    * {@code collectionComplementPartitionToPack}, have been
    * correctly sized, as well as the list of sets {@code basisSets}. For how to appropriately size the partition matrices, see
    * {@link #sizePartitionMatrices(DMatrixRMaj, Set[])}.
    * </p>
    *
    * @param regressor                           the regressor matrix to be partitioned. Not modified.
    * @param basisSets                           the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the regressor. Not modified.
    * @param collectionPartitionToPack           the matrix in which the regressor columns corresponding to the spatial inertia bases in the list of basis sets
    *                                            are stored. Modified.
    * @param collectionComplementPartitionToPack the matrix in which the remaining regressor columns are stored. Modified.
    * @param checkInputs                         whether to perform sanity sizing checks on the inputs.
    */
   public static void partitionRegressor(DMatrixRMaj regressor,
                                         Set<SpatialInertiaBasisOption>[] basisSets,
                                         DMatrixRMaj collectionPartitionToPack,
                                         DMatrixRMaj collectionComplementPartitionToPack,
                                         boolean checkInputs)
   {
      if (checkInputs)
      {
         // Both partition matrices must have the same number of rows as the regressor
         if (collectionPartitionToPack.numRows != regressor.numRows || collectionComplementPartitionToPack.numRows != regressor.numRows)
            throw new IllegalArgumentException("The partition matrices must have the same number of rows as the regressor.");

         // The number of columns over both partition matrices must sum to the number of columns in the regressor
         if (collectionPartitionToPack.numCols + collectionComplementPartitionToPack.numCols != regressor.numCols)
            throw new IllegalArgumentException("The number of columns over both partition matrices must sum to the number of columns in the regressor.");

         // The total number of entries in the list of basis sets must sum to the number of columns in the collected partition matrix
         int totalNumberOfBasisEntries = 0;
         for (Set<SpatialInertiaBasisOption> basisSet : basisSets)
            totalNumberOfBasisEntries += basisSet.size();
         if (totalNumberOfBasisEntries != collectionPartitionToPack.numCols)
            throw new IllegalArgumentException("The total number of entries in the list of basis sets must sum to the number of columns in the regressor.");
      }

      int collectionPartitionIndex = 0;
      int collectionComplementPartitionIndex = 0;

      for (int i = 0; i < basisSets.length; ++i)
      {
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            // NOTE: we use the ordinal of the basis option to index the correct regressor column -- this is how we effectively translate between an (unordered)
            // set and an ordered list of basis options
            int regressorColumnIndex = (i * PARAMETERS_PER_RIGID_BODY) + option.ordinal();
            if (basisSets[i].contains(option))
            {
               MatrixMissingTools.setMatrixColumn(collectionPartitionToPack, collectionPartitionIndex, regressor, regressorColumnIndex);
               collectionPartitionIndex += 1;
            }
            else
            {
               MatrixMissingTools.setMatrixColumn(collectionComplementPartitionToPack, collectionComplementPartitionIndex, regressor, regressorColumnIndex);
               collectionComplementPartitionIndex += 1;
            }
         }
      }
   }

   /**
    * Partition a vector into two vectors, one vector containing the entries corresponding to the spatial inertia bases in the list of
    * {@code SpatialInertiaBasisOption} sets, and the other vector containing the remaining entries.
    * <p>
    * NOTE: This method assumes that the two partition vectors, {@code collectionPartitionToPack} and {@code collectionComplementPartitionToPack}, have been
    * correctly sized. See {@link #sizePartitionVectors(Set[])}.
    * </p>
    *
    * @param vector                              the vector to be partitioned. Not modified.
    * @param basisSets                           the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the vector. Not modified.
    * @param collectionPartitionToPack           the vector in which the entries corresponding to the spatial inertia bases in the list of basis sets are
    *                                            stored. Modified.
    * @param collectionComplementPartitionToPack the vector in which the remaining entries are stored. Modified.
    */
   public static void partitionVector(DMatrixRMaj vector,
                                      Set<SpatialInertiaBasisOption>[] basisSets,
                                      DMatrixRMaj collectionPartitionToPack,
                                      DMatrixRMaj collectionComplementPartitionToPack)
   {
      partitionVector(vector, basisSets, collectionPartitionToPack, collectionComplementPartitionToPack, false);
   }

   /**
    * Partition a vector into two vectors, one vector containing the entries corresponding to the spatial inertia bases in the list of
    * {@code SpatialInertiaBasisOption} sets, and the other vector containing the remaining entries.
    * <p>
    * NOTE: This method will perform checks to ensure that the two partition vectors, {@code collectionPartitionToPack} and
    * {@code collectionComplementPartitionToPack}, have been correctly sized, as well as the list of sets {@code basisSets}. For how to appropriately size the
    * partition vectors, see {@link #sizePartitionVectors(Set[])}.
    * </p>
    *
    * @param vector                              the vector to be partitioned. Not modified.
    * @param basisSets                           the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the vector. Not modified.
    * @param collectionPartitionToPack           the vector in which the entries corresponding to the spatial inertia bases in the list of basis sets are
    *                                            stored. Modified.
    * @param collectionComplementPartitionToPack the vector in which the remaining entries are stored. Modified.
    * @param checkInputs                         whether to perform sanity sizing checks on the inputs.
    */
   public static void partitionVector(DMatrixRMaj vector,
                                      Set<SpatialInertiaBasisOption>[] basisSets,
                                      DMatrixRMaj collectionPartitionToPack,
                                      DMatrixRMaj collectionComplementPartitionToPack,
                                      boolean checkInputs)
   {
      if (checkInputs)
      {
         // The sum of the number of entries in the partitioned vectors must sum to the number of entries in the original vector
         if (collectionPartitionToPack.getNumElements() + collectionComplementPartitionToPack.getNumElements() != vector.getNumElements())
            throw new IllegalArgumentException(
                  "The sum of the number of entries in the partitioned vectors must sum to the number of entries in the original vector.");
         // The total number of entries in the list of basis sets must sum to the number of entries in the collected partition vector
         int totalNumberOfBasisEntries = 0;
         for (Set<SpatialInertiaBasisOption> basisSet : basisSets)
            totalNumberOfBasisEntries += basisSet.size();
         if (totalNumberOfBasisEntries != collectionPartitionToPack.getNumElements())
            throw new IllegalArgumentException("The total number of entries in the list of basis sets must sum to the number of entries in the original vector.");
      }

      int collectionPartitionIndex = 0;
      int collectionComplementPartitionIndex = 0;

      for (int i = 0; i < basisSets.length; ++i)
      {
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            int vectorIndex = (i * PARAMETERS_PER_RIGID_BODY) + option.ordinal();
            if (basisSets[i].contains(option))
            {
               collectionPartitionToPack.set(collectionPartitionIndex, vector.get(vectorIndex));
               collectionPartitionIndex += 1;
            }
            else
            {
               collectionComplementPartitionToPack.set(collectionComplementPartitionIndex, vector.get(vectorIndex));
               collectionComplementPartitionIndex += 1;
            }
         }
      }
   }

   /**
    * A helper method for sizing the partition matrices for {@link #partitionRegressor(DMatrixRMaj, Set[], DMatrixRMaj, DMatrixRMaj)}.
    *
    * @param basisSets the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the regressor. Not modified.
    * @return an array containing the sizes (specifically, the number of columns) of the two partition matrices.
    */
   public static int[] sizePartitions(Set<SpatialInertiaBasisOption>[] basisSets)
   {
      int collectionPartitionSize = 0;
      int collectionComplementPartitionSize = 0;

      for (Set<SpatialInertiaBasisOption> basisSet : basisSets)
      {
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            if (basisSet.contains(option))
               collectionPartitionSize += 1;
            else
               collectionComplementPartitionSize += 1;
         }
      }

      return new int[] {collectionPartitionSize, collectionComplementPartitionSize};
   }

   /**
    * A helper method for constructing correctly-sized partition matrices for {@link #partitionRegressor(DMatrixRMaj, Set[], DMatrixRMaj, DMatrixRMaj)}.
    * <p>
    * This method creates garbage. Try to only use it on construction.
    * </p>
    *
    * @param regressor the regressor matrix to be partitioned. Not modified.
    * @param basisSets the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the regressor. Not modified.
    * @return an array containing the two correctly sized partition matrices.
    */
   public static DMatrixRMaj[] sizePartitionMatrices(DMatrixRMaj regressor, Set<SpatialInertiaBasisOption>[] basisSets)
   {
      int[] partitionSizes = sizePartitions(basisSets);

      return new DMatrixRMaj[] {new DMatrixRMaj(regressor.numRows, partitionSizes[0]), new DMatrixRMaj(regressor.numRows, partitionSizes[1])};
   }

   /**
    * A helper method for sizing the partition vectors for {@link #partitionVector(DMatrixRMaj, Set[], DMatrixRMaj, DMatrixRMaj)}.
    * <p>
    * This method creates garbage. Try to only use it on construction.
    * </p>
    *
    * @param basisSets the list of {@code SpatialInertiaBasisOption} sets to use for partitioning the vector. Not modified.
    * @return an array containing the two correctly sized partition vectors.
    */
   public static DMatrixRMaj[] sizePartitionVectors(Set<SpatialInertiaBasisOption>[] basisSets)
   {
      int[] partitionSizes = sizePartitions(basisSets);

      return new DMatrixRMaj[] {new DMatrixRMaj(partitionSizes[0], 1), new DMatrixRMaj(partitionSizes[1], 1)};
   }

   /**
    * Helper method for packing a vector of inertial parameters into a list of rigid bodies.
    *
    * @param basisSets the list of {@code SpatialInertiaBasisOption} representing the inertial parameters to be packed. Not modified.
    * @param vector the inertial parameters to be packed corresponding to the entries of {@code basisSets}. Not modified.
    * @param bodies the rigid bodies to pack the inertial parameters into. Modified.
    */
   public static void packRigidBodies(Set<SpatialInertiaBasisOption>[] basisSets, DMatrix vector, RigidBodyBasics[] bodies)
   {
      int vectorIndex = 0;
      for (int i = 0; i < basisSets.length; ++i)
      {
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            if (basisSets[i].contains(option))
            {
               switch(option)
               {
                  case M:
                     bodies[i].getInertia().setMass(vector.get(vectorIndex, 0));
                     break;
                  default:
                     LogTools.info("Unhandled basis option: " + option);
                     break;
               }
               vectorIndex += 1;
            }
         }
      }
   }
}