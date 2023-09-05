package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.matrixlib.NativeMatrixTools;

import java.util.Arrays;

/**
 * Configures the QP solver to substitute some of the variables being optimize {@code x} to a set of
 * variables {@code y}. The 2 sets of variables are related via the following equation:
 * 
 * <pre>
 * x = G y + g
 * </pre>
 * <p>
 * This is to facilitate the optimization of {@code x} when some of its components are not
 * independent and that the actual number of independent variables, i.e. represented by {@code y},
 * is smaller than the number of elements in {@code x}.
 * </p>
 * <p>
 * The original implementation was for handling four bar linkages which have 4 1-DoF joints but only
 * have overall 1 DoF.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class NativeQPVariableSubstitution implements QPVariableSubstitutionInterface<NativeMatrix>
{
   /** Refers to the number of elements in {@code x} that are to be substituted. */
   private int numberOfVariablesToSubstitute;
   /** Refers to the index of each element in {@code x} that is to be substituted. */
   private int[] variableIndices = new int[4];
   /** Refers to the indices of each element in {@code y} that represent the actuated joints. */
   private TIntArrayList activeIndices = new TIntArrayList(4, -1);
   private TIntArrayList inactiveIndices = new TIntArrayList(4, -1);

   /** Refers to the transformation matrix {@code G}. */
   private final NativeMatrix transformation = new NativeMatrix(4, 1);
   /** Refers to the bias vector {@code g}. */
   private final NativeMatrix bias = new NativeMatrix(4, 1);
   /**
    * When {@code true}, the operations related to concatenating and substituting will be simplified
    * assuming that the bias is zero.
    */
   private boolean ignoreBias = false;

   // For garbage-free operations during the variable substitution and concatenation
   private final NativeMatrix tempA = new NativeMatrix(1, 1);
   private final NativeMatrix tempB = new NativeMatrix(1, 1);
   private final NativeMatrix tempC = new NativeMatrix(1, 1);
   private final NativeMatrix tempD = new NativeMatrix(1, 1);
   private final NativeMatrix tempE = new NativeMatrix(1, 1);

   private final NativeMatrix AinTemp = new NativeMatrix(1, 1);
   private final NativeMatrix binTemp = new NativeMatrix(1, 1);

   public NativeQPVariableSubstitution()
   {
      reset();
   }

   @Override
   public int getNumberOfVariablesToSubstitute()
   {
      return numberOfVariablesToSubstitute;
   }

   @Override
   public NativeMatrix getTransformation()
   {
      return transformation;
   }

   @Override
   public NativeMatrix getBias()
   {
      return bias;
   }

   @Override
   public int[] getVariableIndices()
   {
      return variableIndices;
   }

   @Override
   public TIntArrayList getActiveIndices()
   {
      return activeIndices;
   }

   public void reset()
   {
      numberOfVariablesToSubstitute = 0;
      activeIndices.reset();
      inactiveIndices.reset();
      transformation.reshape(0, 0);
      bias.reshape(0, 0);
   }

   public void reshape(int numberOfVariablesToSubstitute, int numberOfVariablesPostSubstitution)
   {
      this.numberOfVariablesToSubstitute = numberOfVariablesToSubstitute;

      if (variableIndices.length < numberOfVariablesToSubstitute)
         variableIndices = new int[numberOfVariablesToSubstitute];

      activeIndices.reset();
      // FIXME this doesn't inherently zero
      transformation.reshape(numberOfVariablesToSubstitute, numberOfVariablesPostSubstitution);
      bias.reshape(numberOfVariablesPostSubstitution, 1);
   }

   public void setIgnoreBias(boolean ignoreBias)
   {
      this.ignoreBias = ignoreBias;
   }

   /**
    * Combine this variable substitution with the other.
    * 
    * @param other the other substitution to add to this. Not modified.
    */
   public void concatenate(QPVariableSubstitutionInterface<NativeMatrix> other)
   {
      int oldSizeX = numberOfVariablesToSubstitute;
      int oldSizeY = transformation.getNumCols();
      int newSizeX = oldSizeX + other.getNumberOfVariablesToSubstitute();
      int newSizeY = oldSizeY + other.getTransformation().getNumCols();

      if (variableIndices.length < newSizeX)
         variableIndices = Arrays.copyOf(variableIndices, newSizeX);

      for (int i = 0; i < other.getNumberOfVariablesToSubstitute(); i++)
      {
         variableIndices[i + oldSizeX] = other.getVariableIndices()[i];
      }

      for (int i = 0; i < other.getActiveIndices().size(); i++)
      {
         activeIndices.add(other.getActiveIndices().get(i) + oldSizeX);
      }

      tempA.set(transformation);
      transformation.reshape(newSizeX, newSizeY);
      transformation.zero();
      transformation.insert(tempA, 0, 0);
      transformation.insert(other.getTransformation(), oldSizeX, oldSizeY);

      if (!ignoreBias)
      {
         // Since bias is a vector, it is safe to reshape it and trust that the coefficients won't be shifted.
         bias.reshape(newSizeX, 1);
         bias.zero();
         bias.insert(other.getBias(), oldSizeX, oldSizeY);
      }

      numberOfVariablesToSubstitute = newSizeX;
   }

   /**
    * When this substitution is empty, this signifies that there is no substitution to be applied in
    * the QP.
    * 
    * @return {@code true} if there's no substitution to be performed, {@code false} otherwise.
    */
   public boolean isEmpty()
   {
      return numberOfVariablesToSubstitute == 0;
   }

   /**
    * Gets the indices of the variables that should be removed/disabled from the optimization.
    * 
    * @return the indices of the inactive variables.
    */
   public TIntArrayList getInactiveIndices()
   {
      /*
       * After substitution, the number of variables for this part of this problem goes from
       * transformation.getNumRows() down to transformation.getNumCols(). We will use the
       * (transformation.getNumCols()) first variables only after substitution, we want to disable the
       * other.
       */
      int numberOfInactiveIndices = transformation.getNumRows() - transformation.getNumCols();
      inactiveIndices.reset();

      for (int i = 0; i < numberOfInactiveIndices; i++)
      {
         inactiveIndices.add(variableIndices[transformation.getNumCols() + i]);
      }

      return inactiveIndices;
   }

   /**
    * Performs a variable substitution to the objective function of a QP problem:
    * 
    * <pre>
    * min<sub>x</sub> 0.5 * x<sup>T</sup> H x + f<sup>T</sup>x
    * s.t.
    *    A<sub>in</sub> x &leq; b<sub>in</sub>
    *    A<sub>eq</sub> x = b<sub>eq</sub>
    *    x &geq; x<sub>min</sub>
    *    x &leq; x<sub>max</sub>
    * </pre>
    * <p>
    * Here's the transformation performed:
    * 
    * <pre>
    * H = G<sup>T</sup> H G
    * f = G<sup>T</sup> G<sup>T</sup> H g
    * where:
    *    - H is symmetric
    *    - G<sub>nRows</sub> > G<sub>nCols</sub>
    * </pre>
    * 
    * Main specificity here is to preserve the dimensions of {@code H} and {@code f}, instead of
    * removing rows and columns they will be padded with zeros.
    * </p>
    * 
    * @param H the N-by-N matrix as shown above. Modified.
    * @param f the N-by-1 vector as shown above. Modified.
    */
   public void applySubstitutionToObjectiveFunction(NativeMatrix H, NativeMatrix f)
   {
      // Changing notation to simplify the rest.
      NativeMatrix G = transformation;
      NativeMatrix g = bias;
      NativeMatrix sub_H = tempA;
      NativeMatrix sub_f = tempB;

      /*
       * @formatter:off 
       * Operation in matrix form assuming the variableIndices are successive:
       * 
       *      / 1 0  0 \ / H11 H12 H13 \ / 1 0 0 \
       * H* = | 0 GT 0 | | H12 H22 H23 | | 0 G 0 |
       *      \ 0 0  1 / \ H13 H23 H33 / \ 0 0 1 /
       *      /  H11     H12*G   H13   \
       *    = | GT*H12 GT*H22*G GT*H23 |
       *      \  H13     H23*G   H33   /
       * @formatter:on
       */

      // 1. Compute G^T*H
      // Extracting the rows from H to facilitate the operation.
      NativeMatrix GTH = tempC;
      sub_H.reshape(numberOfVariablesToSubstitute, H.getNumCols());
      sub_H.zero();
      NativeMatrixTools.extractRows(H, variableIndices, numberOfVariablesToSubstitute, sub_H, 0);
      GTH.multTransA(G, sub_H);

      // Re-inserting the rows into H. Since there are less rows than when we started, we'll add padding with zeros to prevent changing H size.
      for (int i = 0; i < numberOfVariablesToSubstitute; i++)
      {
         int rowH = variableIndices[i];

         if (i < GTH.getNumRows())
            H.insert(GTH, i, i + 1, 0, H.getNumCols(), rowH, 0);
         else
            H.zeroRow(rowH);
      }

      // 2. Compute f = G^T*f + G^T*H*g (ignoring the size mismatch between the (H, f) and (G, g)).
      NativeMatrix GTf = tempD;
      sub_f.reshape(G.getNumRows(), 1);
      sub_f.zero();
      NativeMatrixTools.extract(f, variableIndices, numberOfVariablesToSubstitute, sub_f);
      // G^T*f
      GTf.multTransA(G, sub_f);

      if (ignoreBias)
      {
         sub_f.set(GTf);
      }
      else
      {
         // G^T*H*g
         NativeMatrix sub_GTH = tempE;
         sub_GTH.reshape(G.getNumCols(), numberOfVariablesToSubstitute);
         sub_GTH.zero();
         NativeMatrixTools.extractColumns(GTH, variableIndices, numberOfVariablesToSubstitute, sub_GTH, 0);
         sub_f.mult(sub_GTH, g);
         // G^T*f + G^T*H*g
         sub_f.addEquals(GTf);
      }

      // Re-inserting the elements into f. Since there are less elements than when we started, we'll add padding with zeros to prevent changing f size.
      for (int i = 0; i < numberOfVariablesToSubstitute; i++)
      {
         int row_f = variableIndices[i];

         if (i < sub_f.getNumRows())
            f.setElement(row_f, 0, sub_f, i, 0);
         else
            f.set(row_f, 0, 0.0);
      }

      // 3. Compute H*G
      // Extracting the columns from H to facilitate the operation.
      NativeMatrix HG = tempC;
      sub_H.reshape(H.getNumRows(), numberOfVariablesToSubstitute);
      sub_H.zero();
      NativeMatrixTools.extractColumns(H, variableIndices, numberOfVariablesToSubstitute, sub_H, 0);
      HG.mult(sub_H, G);

      // Re-inserting the columns into H. Since there are less columns than when we started, we'll add padding with zeros to prevent changing H size.
      for (int i = 0; i < numberOfVariablesToSubstitute; i++)
      {
         int colH = variableIndices[i];

         if (i < HG.getNumCols())
            H.insert(HG, 0, H.getNumRows(), i, i + 1, 0, colH);
         else
            H.zeroRow(colH);
      }
   }

   /**
    * Performs a variable substitution to the equality constraint of a QP problem:
    * 
    * <pre>
    * min<sub>x</sub> 0.5 * x<sup>T</sup> H x + f<sup>T</sup>x
    * s.t.
    *    A<sub>in</sub> x &leq; b<sub>in</sub>
    *    A<sub>eq</sub> x = b<sub>eq</sub>
    *    x &geq; x<sub>min</sub>
    *    x &leq; x<sub>max</sub>
    * </pre>
    * <p>
    * Here's the transformation performed:
    * 
    * <pre>
    * A = A G
    * b = b - A g
    * where:
    *    - G<sub>nRows</sub> > G<sub>nCols</sub>
    * </pre>
    * 
    * Main specificity here is to preserve the dimensions of {@code A} and {@code b}, instead of
    * removing rows and columns they will be padded with zeros.
    * </p>
    * 
    * @param A the M-by-N matrices <tt>A<sub>in</sub></tt> or <tt>A<sub>eq</sub></tt> as shown above.
    *          Modified.
    * @param b the M-by-1 vectors <tt>b<sub>in</sub></tt> or <tt>b<sub>eq</sub></tt> as shown above.
    *          Modified.
    */
   public void applySubstitutionToLinearConstraint(NativeMatrix A, NativeMatrix b)
   {
      if (A.getNumRows() == 0)
         return;

      // Changing notation to simplify the rest.
      NativeMatrix G = transformation;
      NativeMatrix g = bias;
      NativeMatrix sub_A = tempA;

      // 1. Compute A*G
      NativeMatrix AG = tempC;
      sub_A.reshape(A.getNumRows(), numberOfVariablesToSubstitute);
      sub_A.zero();
      NativeMatrixTools.extractColumns(A, variableIndices, numberOfVariablesToSubstitute, sub_A, 0);
      AG.mult(sub_A, G);

      // Re-inserting the columns into A. Since there are less columns than when we started, we'll add padding with zeros to prevent changing A size.
      for (int i = 0; i < numberOfVariablesToSubstitute; i++)
      {
         int colA = variableIndices[i];

         if (i < AG.getNumCols())
            A.insert(AG, 0, A.getNumRows(), i, i + 1, 0, colA);
         else
            A.zeroRow(colA);
      }

      if (!ignoreBias)
      {
         // 2. Compute b-A*g
         NativeMatrix Ag = tempB;
         Ag.mult(sub_A, g);

         for (int i = 0; i < numberOfVariablesToSubstitute; i++)
         {
            int row_b = variableIndices[i];
            b.set(row_b, 0, b.get(row_b, 0) - Ag.get(i, 0));
         }
      }
   }

   /**
    * Performs a variable substitution to the variables bounds of a QP problem:
    * 
    * <pre>
    * min<sub>x</sub> 0.5 * x<sup>T</sup> H x + f<sup>T</sup>x
    * s.t.
    *    A<sub>in</sub> x &leq; b<sub>in</sub>
    *    A<sub>eq</sub> x = b<sub>eq</sub>
    *    x &geq; x<sub>min</sub>
    *    x &leq; x<sub>max</sub>
    * </pre>
    * <p>
    * The transformation of the variable bounds is as follows:
    * 
    * <pre>
    * x &geq; x<sub>min</sub> &Rightarrow; G y &geq; x<sub>min</sub> - g
    * x &leq; x<sub>max</sub> &Rightarrow; G y &leq; x<sub>max</sub> - g
    * 
    * where:
    *  - x is the original variable vector.
    *  - y is the new variable vector after substitution.
    * </pre>
    * 
    * As shown above, applying the substitution to the variable bounds also modifying the formulation
    * itself such that it has to be moved to the QP's inequality constraint set. So after substitution,
    * <tt>A<sub>in</sub></tt> and <tt>b<sub>in</sub></tt> are augmented to include the variable bounds
    * only for the subset of {@code x} that has been substituted. The portion of
    * <tt>x<sub>min</sub></tt> and <tt>x<sub>max</sub></tt> affected by the substitution will be zeroed
    * out.
    * </p>
    * 
    * @param xMin the lower bound. Modified.
    * @param xMax the upper bound. Modified.
    * @param Ain  the M-by-N matrix as shown above. Modified.
    * @param bin  the M-by-1 vector as shown above. Modified.
    */
   public void applySubstitutionToBounds(NativeMatrix xMin, NativeMatrix xMax, NativeMatrix Ain, NativeMatrix bin)
   {
      // First verify that the bounds are actually set:
      boolean areBoundsSet = false;

      // Assuming that if at least one bound is set, that all are.
      // Consider improving this method to only consider set bounds.
      for (int i = 0; i < activeIndices.size(); i++)
      {
         int variableIndex = variableIndices[activeIndices.get(i)];

         if (Double.isFinite(xMin.get(variableIndex, 0)) || Double.isFinite(xMax.get(variableIndex, 0)))
         {
            areBoundsSet = true;
            break;
         }
      }

      if (!areBoundsSet)
         return;

      // Changing notation to simplify the rest.
      NativeMatrix G = transformation;
      NativeMatrix g = bias;

      int offset_min = Ain.getNumRows();
      int offset_max = offset_min + activeIndices.size();

      AinTemp.set(Ain);
      binTemp.set(Ain);
      Ain.reshape(Ain.getNumRows() + 2 * activeIndices.size(), Ain.getNumCols());
      bin.reshape(bin.getNumRows() + 2 * activeIndices.size(), 1);
      Ain.insert(AinTemp, 0, 0);
      bin.insert(binTemp, 0, 0);

      for (int i = 0; i < activeIndices.size(); i++)
      {
         int rowAmin = i + offset_min;
         int rowAmax = i + offset_max;
         // Make sure the new rows do not contain old values.
         Ain.zeroRow(rowAmin);
         Ain.zeroRow(rowAmax);

         int rowG = activeIndices.get(i);

         for (int colG = 0; colG < G.getNumCols(); colG++)
         {
            int colA = variableIndices[colG];
            Ain.unsafe_set(rowAmin, colA, -(G.unsafe_get(rowG, colG))); // The lower bound is negated to switch from greater-or-equal to less-or-equal as formulated in the QP.
            Ain.unsafe_set(rowAmax, colA, G.unsafe_get(rowG, colG));
         }
      }

      for (int i = 0; i < activeIndices.size(); i++)
      {
         int row_g = activeIndices.get(i);
         int row_x = variableIndices[row_g];
         int row_b_min = i + offset_min;
         int row_b_max = i + offset_max;

         if (ignoreBias)
         {
            bin.set(row_b_min, 0, -(xMin.get(row_x, 0))); // The lower bound is negated to switch from greater-or-equal to less-or-equal as formulated in the QP.
            bin.setElement(row_b_max, 0, xMax, row_x, 0);
         }
         else
         {
            bin.set(row_b_min, 0, -(xMin.get(row_x, 0) - g.get(row_g, 0))); // The lower bound is negated to switch from greater-or-equal to less-or-equal as formulated in the QP.
            bin.set(row_b_max, 0, xMax.get(row_x, 0) - g.get(row_g, 0));
         }
      }

      for (int variableIndex : variableIndices)
      { // Disable indices of xMin and xMax
         xMin.set(variableIndex, 0, Double.NEGATIVE_INFINITY);
         xMax.set(variableIndex, 0, Double.POSITIVE_INFINITY);
      }
   }

   /**
    * Assuming the QP problem was configured with the variable substitution this class represents, this
    * method takes in the solution with the substituted variables, and undo the substitution.
    * <p>
    * When passed in, the argument is assumed to represent {@code y} (variables after substitution),
    * this method applies the following relation to retrieve {@code x}:
    * 
    * <pre>
    * x = G y + g
    * </pre>
    * </p>
    * 
    * @param x the QP's solution for which the substitution is to be undone. Modified.
    */
   public void removeSubstitutionToSolution(NativeMatrix x)
   {
      // Changing notation to simplify the rest.
      NativeMatrix G = transformation;
      NativeMatrix g = bias;
      NativeMatrix sub_y = tempA;
      NativeMatrix sub_x = tempB;

      sub_y.reshape(G.getNumCols(), 1);
      sub_y.zero();

      for (int i = 0; i < G.getNumCols(); i++)
      {
         sub_y.setElement(i, 0, x, variableIndices[i], 0);
      }

      sub_x.mult(G, sub_y);
      if (!ignoreBias)
         sub_x.addEquals(g);

      for (int i = 0; i < sub_x.getNumRows(); i++)
      {
         x.setElement(variableIndices[i], 0, sub_x, i, 0);
      }
   }
}
