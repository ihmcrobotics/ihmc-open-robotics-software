package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.Arrays;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * Configures the QP solver to substitute some of the variables being optimize {@code x} to to set
 * of variables {@code y}. The 2 sets of variables are related via the following equation:
 * 
 * <pre>
 * x = G y + g
 * </pre>
 * <p>
 * This is to facilitate the optimization of {@code x} when some of its components are not
 * independent and that the actual number of independent variables, i.e. represented by {@code y},
 * is smaller than the number of element in {@code x}.
 * </p>
 * <p>
 * The original implementation was for handling four bar linkages which have 4 1-DoF joints but only
 * have overall 1 DoF.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class QPVariableSubstitution
{
   /** Refers to the number of elements in {@code x} that are to be substituted. */
   public int numberOfVariablesToSubstitute;
   /** Refers to the index of each element in {@code x} that is to be substituted. */
   public int[] variableIndices = new int[4];

   /** Refers to the transformation matrix {@code G}. */
   public final DMatrixRMaj transformation = new DMatrixRMaj(4, 1);
   /** Refers to the bias vector {@code g}. */
   public final DMatrixRMaj bias = new DMatrixRMaj(4, 1);

   // For garbage-free operations
   private final DMatrixRMaj temp = new DMatrixRMaj(1, 1);

   public QPVariableSubstitution()
   {
   }

   public void reset()
   {
      numberOfVariablesToSubstitute = 0;
   }

   public void reshape(int numberOfVariablesToSubstitute, int numberOfVariablesPostSubstitution)
   {
      this.numberOfVariablesToSubstitute = numberOfVariablesToSubstitute;

      if (variableIndices.length < numberOfVariablesToSubstitute)
         variableIndices = new int[numberOfVariablesToSubstitute];

      transformation.reshape(numberOfVariablesToSubstitute, numberOfVariablesPostSubstitution);
      bias.reshape(numberOfVariablesPostSubstitution, 1);
   }

   public void concatenate(QPVariableSubstitution other)
   {
      int oldSizeX = numberOfVariablesToSubstitute;
      int oldSizeY = transformation.getNumCols();
      int newSizeX = oldSizeX + other.numberOfVariablesToSubstitute;
      int newSizeY = oldSizeY + other.transformation.getNumCols();

      if (variableIndices.length < newSizeX)
         variableIndices = Arrays.copyOf(variableIndices, newSizeX);

      for (int i = 0; i < other.numberOfVariablesToSubstitute; i++)
      {
         variableIndices[i + oldSizeX] = other.variableIndices[i];
      }

      temp.set(transformation);
      transformation.reshape(newSizeX, newSizeY);
      CommonOps_DDRM.insert(temp, transformation, 0, 0);
      CommonOps_DDRM.insert(other.transformation, transformation, oldSizeX, oldSizeY);

      // Since bias is a vector, it is safe to reshape it and trust that the coefficients won't be shifted.
      bias.reshape(newSizeX, 1);
      CommonOps_DDRM.insert(other.bias, bias, oldSizeX, 0);

      numberOfVariablesToSubstitute = newSizeX;
   }

   public boolean isEmpty()
   {
      return numberOfVariablesToSubstitute == 0;
   }
}
