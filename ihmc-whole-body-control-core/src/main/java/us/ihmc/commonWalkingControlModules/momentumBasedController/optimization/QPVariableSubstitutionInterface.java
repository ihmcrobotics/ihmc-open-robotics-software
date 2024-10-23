package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.MatrixTools;

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
public interface QPVariableSubstitutionInterface<D extends DMatrix>
{
   void reset();

   void reshape(int numberOfVariablesToSubstitute, int numberOfVariablesPostSubstitution);

   void setIgnoreBias(boolean ignoreBias);

   int getNumberOfVariablesToSubstitute();

   D getTransformation();

   D getBias();

   int[] getVariableIndices();

   TIntArrayList getActiveIndices();

   /**
    * Combine this variable substitution with the other.
    * 
    * @param other the other substitution to add to this. Not modified.
    */
   void concatenate(QPVariableSubstitutionInterface<D> other);

   /**
    * When this substitution is empty, this signifies that there is no substitution to be applied in
    * the QP.
    * 
    * @return {@code true} if there's no substitution to be performed, {@code false} otherwise.
    */
   boolean isEmpty();

   /**
    * Gets the indices of the variables that should be removed/disabled from the optimization.
    * 
    * @return the indices of the inactive variables.
    */
   TIntArrayList getInactiveIndices();

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
   void applySubstitutionToObjectiveFunction(D H, D f);

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
   void applySubstitutionToLinearConstraint(D A, D b);

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
   void applySubstitutionToBounds(D xMin, D xMax, D Ain, D bin);

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
   void removeSubstitutionToSolution(D x);
}
