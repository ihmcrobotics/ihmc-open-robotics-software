package us.ihmc.stateEstimation.invariant_estimator;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * State for an SE_k(3) invariant filter: a group element X together with its covariance P.
 * Biases are intentionally excluded — an upstream joint-space EKF is responsible for those.
 *
 * <p><b>Group element</b> (an n×n {@link DMatrixRMaj}, n = 5 + N for N contacts), with the
 * named "base + contacts" column layout:
 * <pre>
 *   rows/cols 0..2 : rotation R
 *   column 3       : base velocity   v
 *   column 4       : base position   p
 *   columns 5..4+N : contact positions p_c0 … p_c(N-1)
 *   bottom-right   : identity block
 * </pre>
 * This corresponds to k = 2 + N translation columns in {@link SEK3_Utils}.</p>
 *
 * <p><b>Covariance</b> P is an m×m {@link DMatrixRMaj} (m = 9 + 3N) in the tangent ordering
 * {@code [δφ; δv; δp; δp_c0; …]}, matching the adjoint dimension. Tangent block offsets are
 * given by the {@code *TangentIndex} accessors.</p>
 */
public class InvariantState
{
   /** Matrix column of the base velocity vector v. */
   private static final int BASE_VELOCITY_COLUMN = 3;
   /** Matrix column of the base position vector p. */
   private static final int BASE_POSITION_COLUMN = 4;
   /** Matrix column of the first contact position p_c0. */
   private static final int FIRST_CONTACT_COLUMN = 5;

   private final int numberOfContacts;
   private final DMatrixRMaj groupElement;
   private final DMatrixRMaj covariance;

   /**
    * Creates a state with the given number of contacts, X = identity and P = 0.
    *
    * @param numberOfContacts the number of contact columns N (≥ 0).
    */
   public InvariantState(int numberOfContacts)
   {
      if (numberOfContacts < 0)
         throw new IllegalArgumentException("numberOfContacts must be >= 0, was " + numberOfContacts);

      this.numberOfContacts = numberOfContacts;

      int k = 2 + numberOfContacts; // base velocity + base position + contacts
      int n = 3 + k;                // group matrix size
      int m = 3 + 3 * k;            // tangent / covariance size

      groupElement = new DMatrixRMaj(n, n);
      CommonOps_DDRM.setIdentity(groupElement);

      covariance = new DMatrixRMaj(m, m); // zero by default
   }

   /** @return the number of contact columns N. */
   public int getNumberOfContacts()
   {
      return numberOfContacts;
   }

   /** @return the group-element size n = 5 + N. */
   public int getGroupSize()
   {
      return groupElement.getNumRows();
   }

   /** @return the tangent/covariance size m = 9 + 3N. */
   public int getTangentSize()
   {
      return covariance.getNumRows();
   }

   /** @return the live group-element matrix X (n×n). Mutating it mutates the state. */
   public DMatrixRMaj getGroupElement()
   {
      return groupElement;
   }

   /** @return the live covariance matrix P (m×m). Mutating it mutates the state. */
   public DMatrixRMaj getCovariance()
   {
      return covariance;
   }

   /** Resets X to identity (R = I, all translation columns zero). */
   public void setToIdentity()
   {
      CommonOps_DDRM.setIdentity(groupElement);
   }

   /**
    * Packs the rotation R (top-left 3×3 block of X) into {@code rotationToPack}.
    *
    * @param rotationToPack the rotation to pack the result into. Modified.
    */
   public void getRotation(RotationMatrixBasics rotationToPack)
   {
      rotationToPack.set(groupElement.get(0,0),groupElement.get(0,1),groupElement.get(0,2),
                         groupElement.get(1,0),groupElement.get(1,1),groupElement.get(1,2),
                         groupElement.get(2,0),groupElement.get(2,1),groupElement.get(2,2));
   }

   /**
    * Sets the rotation block R of X.
    *
    * @param rotation the rotation to copy in. Not modified.
    */
   public void setRotation(RotationMatrixReadOnly rotation)
   {
      groupElement.set(0, 0, rotation.getM00());
      groupElement.set(0, 1, rotation.getM01());
      groupElement.set(0, 2, rotation.getM02());
      groupElement.set(1, 0, rotation.getM10());
      groupElement.set(1, 1, rotation.getM11());
      groupElement.set(1, 2, rotation.getM12());
      groupElement.set(2, 0, rotation.getM20());
      groupElement.set(2, 1, rotation.getM21());
      groupElement.set(2, 2, rotation.getM22());
   }

   /**
    * Packs the base velocity v (column 3 of X) into {@code velocityToPack}.
    *
    * @param velocityToPack the vector to pack the result into. Modified.
    */
   public void getBaseVelocity(Vector3DBasics velocityToPack)
   {
      packColumn(BASE_VELOCITY_COLUMN, velocityToPack);
   }

   /**
    * Sets the base velocity v (column 3 of X).
    *
    * @param velocity the value to copy in. Not modified.
    */
   public void setBaseVelocity(Tuple3DReadOnly velocity)
   {
      setColumn(BASE_VELOCITY_COLUMN, velocity);
   }

   /**
    * Packs the base position p (column 4 of X) into {@code positionToPack}.
    *
    * @param positionToPack the vector to pack the result into. Modified.
    */
   public void getBasePosition(Vector3DBasics positionToPack)
   {
      packColumn(BASE_POSITION_COLUMN, positionToPack);
   }

   /**
    * Sets the base position p (column 4 of X).
    *
    * @param position the value to copy in. Not modified.
    */
   public void setBasePosition(Tuple3DReadOnly position)
   {
      setColumn(BASE_POSITION_COLUMN, position);
   }

   /**
    * Packs contact position p_ci (column 5 + i of X) into {@code positionToPack}.
    *
    * @param contactIndex   the contact index i in [0, N).
    * @param positionToPack the vector to pack the result into. Modified.
    */
   public void getContactPosition(int contactIndex, Vector3DBasics positionToPack)
   {
     checkContactIndex(contactIndex);
     packColumn(FIRST_CONTACT_COLUMN + contactIndex, positionToPack);
   }

   /**
    * Sets contact position p_ci (column 5 + i of X).
    *
    * @param contactIndex the contact index i in [0, N).
    * @param position     the value to copy in. Not modified.
    */
   public void setContactPosition(int contactIndex, Tuple3DReadOnly position)
   {
      checkContactIndex(contactIndex);
      setColumn(FIRST_CONTACT_COLUMN + contactIndex, position);
   }

   /** @return the tangent-space start index of the rotation block (0). */
   public int rotationTangentIndex()
   {
      return 0;
   }

   /** @return the tangent-space start index of the base-velocity block. */
   public int baseVelocityTangentIndex()
   {
      return 3;
   }

   /** @return the tangent-space start index of the base-position block. */
   public int basePositionTangentIndex()
   {
      return 6;
   }

   /**
    * @param contactIndex the contact index i in [0, N).
    * @return the tangent-space start index of contact i's block.
    */
   public int contactTangentIndex(int contactIndex)
   {
      checkContactIndex(contactIndex);
      return 9 + 3 * contactIndex;
   }

   /** Packs the top 3 entires of the given matrix column into a tuple. */
   private void packColumn(int matrixColumn, Vector3DBasics tupleToPack)
   {
      tupleToPack.set(groupElement.get(0,matrixColumn),
                      groupElement.get(1,matrixColumn),
                      groupElement.get(2,matrixColumn));
   }

   /** Writes a tuple into the top 3 entries of the given matrix column. */
   private void setColumn(int matrixColumn, Tuple3DReadOnly tuple)
   {
      groupElement.set(0, matrixColumn, tuple.getX());
      groupElement.set(1, matrixColumn, tuple.getY());
      groupElement.set(2, matrixColumn, tuple.getZ());
   }

   // Validates that a contact index is in [0, numberOfContacts]
   private void checkContactIndex(int contactIndex)
   {
      if (contactIndex < 0 || contactIndex >= numberOfContacts)
         throw new IndexOutOfBoundsException("contactIndex must be in [0, " + numberOfContacts + "), was " + contactIndex);
   }
}
