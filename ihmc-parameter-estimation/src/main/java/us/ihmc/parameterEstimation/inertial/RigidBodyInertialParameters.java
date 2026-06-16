package us.ihmc.parameterEstimation.inertial;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

/**
 * A {@link RigidBodyInertialParameters} keeps track of a rigid body's inertial parameters in several representations. A rigid body can usually be described by
 * ten parameters.
 * <p>
 * The first representation is the familiar mass / first moment of mass / moment of inertia representation (see {@link #parameterVectorPiBasis} for how we order
 * the parameters). This representation of the inertial parameters of a rigid body shows up linearly in the inverse dynamics, meaning that one can use linear
 * least squares methods to get estimates of the parameters.
 * </p>
 * <p>
 * The second representation is from Rucker and Wensing (2022): "A smooth parameterization of rigid body inertia" (link to internal confluence page with PDF:
 * <a href="https://confluence.ihmc.us/display/OSS/Online+Parameter+Estimation">LINK</a>). Unfortunately, this representation loses the nice linear structure, but
 * it has the advantage of being fully physically consistent by construction. Essentially, this means that masses are always positive, and inertia matrices always
 * positive definite, rather than the non-physical values one can get when doing estimation with the other representation.
 * </p>
 *
 * @author James Foster
 */
public class RigidBodyInertialParameters
{
   public static final int PARAMETERS_PER_RIGID_BODY = 10;

   /**
    * We include a small offset to the center of mass to avoid numerical issues when the center of mass is at the origin.
    */
   private static final double ZERO_COM_OFFSET_EPS = 1e-9;

   /**
    * This 10 dimensional vector keeps track of the inertial parameters in "Pi" basis -- which is just the usual convention of:
    * <ul>
    *    <li>mass</li>
    *    <li>first moment of mass (center of mass offset multiplied by the mass)</li>
    *    <li>moment of inertia</li>
    * </ul>
    * one will see everywhere in parameter estimation papers.
    * <p>
    * Specifically, we use the following convention to order the Pi basis of the parameter vector:
    * <pre>
    * (m, h<sub>x</sub>, h<sub>y</sub>, h<sub>z</sub>, I<sub>xx</sub>, I<sub>xy</sub>, I<sub>xz</sub>, I<sub>yy</sub>, I<sub>yz</sub>, I<sub>zz</sub>)
    * </pre>
    * where m is the mass, h is the first moment of mass, and I is the moment of inertia.
    * </p>
    */
   private final DMatrixRMaj parameterVectorPiBasis;

   /**
    * This 10 dimensional vector keeps track of the inertial parameters in "Theta" basis -- which is a smooth, non-singular parameterization of "fully physically
    * consistent" rigid bodies (for more information on why that is desirable, see the paper linked at the start of the class documentation). One can think of the
    * parameters in this parameterization being some manipulation of a reference rigid body, where:
    * <ul>
    *    <li>alpha -- density scaling factor of reference body</li>
    *    <li>d<sub>1</sub>, d<sub>2</sub>, d<sub>3</sub> -- length scaling factors of reference body along x, y, z</li>
    *    <li>s<sub>12</sub>, s<sub>13</sub>, s<sub>23</sub> -- shear factors of reference body in xy, xz, yz directions</li>
    *    <li>t<sub>1</sub>, t<sub>2</sub>, t<sub>3</sub> -- translation factors of reference body along x, y, z</li>
    * </ul>
    * <p>
    * Specifically, we use the following convention to order the Theta basis of the parameter vector:
    * <pre>
    * (alpha, d<sub>1</sub>, d<sub>2</sub>, d<sub>3</sub>, s<sub>12</sub>, s<sub>13</sub>, s<sub>23</sub>, t<sub>1</sub>, t<sub>2</sub>, t<sub>3</sub>)
    * </pre>
    * </p>
    */
   private final DMatrixRMaj parameterVectorThetaBasis;

   boolean isPiBasisUpToDate;

   boolean isThetaBasisUpToDate;

   /**
    * Creates a {@code RigidBodyInertialParameters} object from a rigid body's {@code SpatialInertia}, in order to store the inertial parameters
    * in both default mass / first moment of mass / moment of inertia form (Pi basis) and a fully physically consistent form (Theta basis).
    * <p>
    *
    * </p>
    * @param spatialInertia the spatial inertia of the rigid body we want to store the inertial parameters of.
    */
   public RigidBodyInertialParameters(SpatialInertiaReadOnly spatialInertia)
   {
      parameterVectorPiBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      parameterVectorPiBasis.zero();

      double mass = spatialInertia.getMass();

      DMatrixRMaj centerOfMassOffset = new DMatrixRMaj(3, 1);
      spatialInertia.getCenterOfMassOffset().get(centerOfMassOffset);
      CommonOps_DDRM.add(centerOfMassOffset, ZERO_COM_OFFSET_EPS);  // Add a small offset to avoid numerical issues when the center of mass is at the origin

      DMatrixRMaj momentOfInertia = new DMatrixRMaj(3, 3);
      spatialInertia.getMomentOfInertia().get(momentOfInertia);

      // Mass
      parameterVectorPiBasis.set(0, mass);
      // First moment of mass (center of mass offset multiplied by mass)
      CommonOps_DDRM.scale(mass, centerOfMassOffset);
      CommonOps_DDRM.insert(centerOfMassOffset, parameterVectorPiBasis, 1, 0);
      // Moment of inertia (only pull out upper triangular terms of symmetric matrix)
      parameterVectorPiBasis.set(4, momentOfInertia.get(0, 0)); // Ixx
      parameterVectorPiBasis.set(5, momentOfInertia.get(0, 1)); // Ixy
      parameterVectorPiBasis.set(6, momentOfInertia.get(0, 2)); // Ixz
      parameterVectorPiBasis.set(7, momentOfInertia.get(1, 1)); // Iyy
      parameterVectorPiBasis.set(8, momentOfInertia.get(1, 2)); // Iyz
      parameterVectorPiBasis.set(9, momentOfInertia.get(2, 2)); // Izz

      // Construct the parameter vector in theta basis
      parameterVectorThetaBasis = new DMatrixRMaj(PARAMETERS_PER_RIGID_BODY, 1);
      parameterVectorThetaBasis.zero();
      isPiBasisUpToDate = true;
      isThetaBasisUpToDate = false;
      update();
   }

   /**
    * Method for lazily updating parameter representations. Here, the conversions from one basis to another are only performed if the booleans keeping track of
    * whether the respective bases are up-to-date are false. If neither parameter representation is up-to-date, then neither of them can be trusted, so a warning
    * is given to the user and no updates are performed.
    */
   public void update()
   {
      if (!isPiBasisUpToDate && !isThetaBasisUpToDate)
         LogTools.warn("Neither the Pi basis or the Theta basis of the inertial parameter vector is updated -- please update one of them. No operation has been performed.");

      if (!isThetaBasisUpToDate)
      {
         fromPiBasisToThetaBasis();
         isThetaBasisUpToDate = true;
      }
      if (!isPiBasisUpToDate)
      {
         fromThetaBasisToPiBasis();
         isPiBasisUpToDate = true;
      }
   }

   /**
    * Internal method for converting from Pi basis to Theta basis.
    * <p>
    * Implements the inverse of the corrected forward mapping (Eq. (1) of Rucker and Wensing 2026, "Correction
    * to Smooth Parameterization of Rigid-Body Inertia"), i.e. the J = U U^T decomposition.
    * </p>
    */
   private void fromPiBasisToThetaBasis()
   {
      fromPiBasisToThetaBasis(parameterVectorPiBasis, parameterVectorThetaBasis);
   }

   /**
    * Internal method for converting from Theta basis to Pi basis.
    */
   private void fromThetaBasisToPiBasis()
   {
      fromThetaBasisToPiBasis(parameterVectorThetaBasis, parameterVectorPiBasis);
   }

   /**
    * Static method for converting from Pi basis to Theta basis.
    * <p>
    * Implements the inverse of the corrected forward mapping (Eq. (1) of Rucker and Wensing 2026, "Correction
    * to Smooth Parameterization of Rigid-Body Inertia"), i.e. the J = U U^T decomposition.
    * </p>
    *
    * @param parameterVectorPiBasis parameter vector in the default Pi basis.
    * @param parameterVectorThetaBasisToPack the vector in which to pack the equivalent parameter vector in Theta basis.
    */
   public static void fromPiBasisToThetaBasis(DMatrixRMaj parameterVectorPiBasis, DMatrixRMaj parameterVectorThetaBasisToPack)
   {
      // Unpack variables for legibility
      double m = parameterVectorPiBasis.get(0);
      double hx = parameterVectorPiBasis.get(1);
      double hy = parameterVectorPiBasis.get(2);
      double hz = parameterVectorPiBasis.get(3);
      double Ixx = parameterVectorPiBasis.get(4);
      double Ixy = parameterVectorPiBasis.get(5);
      double Ixz = parameterVectorPiBasis.get(6);
      double Iyy = parameterVectorPiBasis.get(7);
      double Iyz = parameterVectorPiBasis.get(8);
      double Izz = parameterVectorPiBasis.get(9);

      // Inverse of the corrected mapping (Eq. (1) of the 2026 correction). Because h = m * t in the corrected
      // mapping, the translation parameters are just the center of mass, and the d/s parameters follow from the
      // center-of-mass-relative (barycentric) inertia. Derived by inverting the forward map term by term.
      double alpha = 0.5 * Math.log(m);
      double t1 = hx / m;
      double t2 = hy / m;
      double t3 = hz / m;

      // Barycentric inertia entries divided by mass (subtract the parallel-axis translation terms).
      double a = Ixx / m - (t2 * t2 + t3 * t3); // = s23^2 + e^(2 d2) + e^(2 d3)
      double b = Iyy / m - (t1 * t1 + t3 * t3); // = s12^2 + s13^2 + e^(2 d1) + e^(2 d3)
      double c = Izz / m - (t1 * t1 + t2 * t2); // = s12^2 + s13^2 + s23^2 + e^(2 d1) + e^(2 d2)
      double dd = Ixy / m + t1 * t2;            // = -s13 s23 - s12 e^(d2)
      double e = Iyz / m + t2 * t3;             // = -s23 e^(d3)
      double f = Ixz / m + t1 * t3;             // = -s13 e^(d3)

      double expD3 = Math.sqrt(0.5 * (a + b - c)); // a + b - c = 2 e^(2 d3)
      double s13 = -f / expD3;
      double s23 = -e / expD3;
      double expD2 = Math.sqrt(a - s23 * s23 - expD3 * expD3);
      double s12 = (-dd - s13 * s23) / expD2;
      double expD1 = Math.sqrt(b - s12 * s12 - s13 * s13 - expD3 * expD3);

      parameterVectorThetaBasisToPack.set(0, alpha);
      parameterVectorThetaBasisToPack.set(1, Math.log(expD1));
      parameterVectorThetaBasisToPack.set(2, Math.log(expD2));
      parameterVectorThetaBasisToPack.set(3, Math.log(expD3));
      parameterVectorThetaBasisToPack.set(4, s12);
      parameterVectorThetaBasisToPack.set(5, s13);
      parameterVectorThetaBasisToPack.set(6, s23);
      parameterVectorThetaBasisToPack.set(7, t1);
      parameterVectorThetaBasisToPack.set(8, t2);
      parameterVectorThetaBasisToPack.set(9, t3);
   }

   /**
    * Static method for converting from Theta basis to Pi basis.
    *
    * @param parameterVectorThetaBasis parameter vector in Rucker and Wensing's Theta basis.
    * @param parameterVectorPiBasisToPack the vector in which to pack the equivalent parameter vector in Pi basis.
    */
   public static void fromThetaBasisToPiBasis(DMatrixRMaj parameterVectorThetaBasis, DMatrixRMaj parameterVectorPiBasisToPack)
   {
      // Unpack variables for legibility
      double alpha = parameterVectorThetaBasis.get(0);
      double d1 = parameterVectorThetaBasis.get(1);
      double d2 = parameterVectorThetaBasis.get(2);
      double d3 = parameterVectorThetaBasis.get(3);
      double s12 = parameterVectorThetaBasis.get(4);
      double s13 = parameterVectorThetaBasis.get(5);
      double s23 = parameterVectorThetaBasis.get(6);
      double t1 = parameterVectorThetaBasis.get(7);
      double t2 = parameterVectorThetaBasis.get(8);
      double t3 = parameterVectorThetaBasis.get(9);

      double expD1 = Math.exp(d1);
      double expD2 = Math.exp(d2);
      double expD3 = Math.exp(d3);
      double squareExpD1 = expD1 * expD1;
      double squareExpD2 = expD2 * expD2;
      double squareExpD3 = expD3 * expD3;
      double squareS12 = MathTools.pow(s12, 2);
      double squareS13 = MathTools.pow(s13, 2);
      double squareS23 = MathTools.pow(s23, 2);
      double squareT1 = MathTools.pow(t1, 2);
      double squareT2 = MathTools.pow(t2, 2);
      double squareT3 = MathTools.pow(t3, 2);

      // Corrected mapping (Eq. (1) of Rucker & Wensing 2026 "Correction to Smooth Parameterization of
      // Rigid-Body Inertia"), consistent with the decomposition J = U U^T (Eq. (7) of the 2022 paper). The
      // earlier code implemented Eq. (9) of the 2022 paper, which was derived from J = U^T U and is thus
      // inconsistent. With the correct mapping the translation parameters are the center-of-mass directly
      // (h = m * t). Packed in the Pi-basis order (m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz).
      parameterVectorPiBasisToPack.set(0, 1.0);                                                              // m
      parameterVectorPiBasisToPack.set(1, t1);                                                               // hx
      parameterVectorPiBasisToPack.set(2, t2);                                                               // hy
      parameterVectorPiBasisToPack.set(3, t3);                                                               // hz
      parameterVectorPiBasisToPack.set(4, squareS23 + squareT2 + squareT3 + squareExpD2 + squareExpD3);      // Ixx
      parameterVectorPiBasisToPack.set(5, -s13 * s23 - t1 * t2 - s12 * expD2);                               // Ixy
      parameterVectorPiBasisToPack.set(6, -t1 * t3 - s13 * expD3);                                           // Ixz
      parameterVectorPiBasisToPack.set(7, squareS12 + squareS13 + squareT1 + squareT3 + squareExpD1 + squareExpD3); // Iyy
      parameterVectorPiBasisToPack.set(8, -t2 * t3 - s23 * expD3);                                           // Iyz
      parameterVectorPiBasisToPack.set(9, squareS12 + squareS13 + squareS23 + squareT1 + squareT2 + squareExpD1 + squareExpD2); // Izz
      CommonOps_DDRM.scale(Math.exp(2 * alpha), parameterVectorPiBasisToPack);
   }

   public static void fromThetaBasisToPiBasisJacobian(DMatrixRMaj parameterVectorThetaBasis, DMatrixRMaj jacobianToPack)
   {
      // Unpack variables for legibility
      double alpha = parameterVectorThetaBasis.get(0);
      double d1 = parameterVectorThetaBasis.get(1);
      double d2 = parameterVectorThetaBasis.get(2);
      double d3 = parameterVectorThetaBasis.get(3);
      double s12 = parameterVectorThetaBasis.get(4);
      double s13 = parameterVectorThetaBasis.get(5);
      double s23 = parameterVectorThetaBasis.get(6);
      double t1 = parameterVectorThetaBasis.get(7);
      double t2 = parameterVectorThetaBasis.get(8);
      double t3 = parameterVectorThetaBasis.get(9);

      // Intermediate variables
      double exp2Alpha = Math.exp(2 * alpha);
      double expD1 = Math.exp(d1);
      double expD2 = Math.exp(d2);
      double expD3 = Math.exp(d3);
      double squareExpD1 = expD1 * expD1;
      double squareExpD2 = expD2 * expD2;
      double squareExpD3 = expD3 * expD3;
      double squareS12 = MathTools.pow(s12, 2);
      double squareS13 = MathTools.pow(s13, 2);
      double squareS23 = MathTools.pow(s23, 2);
      double squareT1 = MathTools.pow(t1, 2);
      double squareT2 = MathTools.pow(t2, 2);
      double squareT3 = MathTools.pow(t3, 2);

      // Analytic Jacobian d(pi)/d(theta) of the corrected forward map (Eq. (1) of the 2026 correction).
      // Rows are the Pi-basis params (m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz); columns are the Theta-basis
      // params (alpha, d1, d2, d3, s12, s13, s23, t1, t2, t3). Every pi_i = e^(2 alpha) * g_i, so the alpha
      // column (0) is 2 * pi_i and other columns are e^(2 alpha) * dg_i.
      CommonOps_DDRM.fill(jacobianToPack, 0.0);

      // Row 0: m = e^(2a)
      jacobianToPack.set(0, 0, 2.0 * exp2Alpha);
      // Row 1: hx = e^(2a) t1
      jacobianToPack.set(1, 0, 2.0 * exp2Alpha * t1);
      jacobianToPack.set(1, 7, exp2Alpha);
      // Row 2: hy = e^(2a) t2
      jacobianToPack.set(2, 0, 2.0 * exp2Alpha * t2);
      jacobianToPack.set(2, 8, exp2Alpha);
      // Row 3: hz = e^(2a) t3
      jacobianToPack.set(3, 0, 2.0 * exp2Alpha * t3);
      jacobianToPack.set(3, 9, exp2Alpha);
      // Row 4: Ixx = e^(2a)(s23^2 + t2^2 + t3^2 + e^(2 d2) + e^(2 d3))
      jacobianToPack.set(4, 0, 2.0 * exp2Alpha * (squareS23 + squareT2 + squareT3 + squareExpD2 + squareExpD3));
      jacobianToPack.set(4, 2, 2.0 * exp2Alpha * squareExpD2);
      jacobianToPack.set(4, 3, 2.0 * exp2Alpha * squareExpD3);
      jacobianToPack.set(4, 6, 2.0 * exp2Alpha * s23);
      jacobianToPack.set(4, 8, 2.0 * exp2Alpha * t2);
      jacobianToPack.set(4, 9, 2.0 * exp2Alpha * t3);
      // Row 5: Ixy = e^(2a)(-s13 s23 - t1 t2 - s12 e^(d2))
      jacobianToPack.set(5, 0, 2.0 * exp2Alpha * (-s13 * s23 - t1 * t2 - s12 * expD2));
      jacobianToPack.set(5, 2, exp2Alpha * (-s12 * expD2));
      jacobianToPack.set(5, 4, exp2Alpha * (-expD2));
      jacobianToPack.set(5, 5, exp2Alpha * (-s23));
      jacobianToPack.set(5, 6, exp2Alpha * (-s13));
      jacobianToPack.set(5, 7, exp2Alpha * (-t2));
      jacobianToPack.set(5, 8, exp2Alpha * (-t1));
      // Row 6: Ixz = e^(2a)(-t1 t3 - s13 e^(d3))
      jacobianToPack.set(6, 0, 2.0 * exp2Alpha * (-t1 * t3 - s13 * expD3));
      jacobianToPack.set(6, 3, exp2Alpha * (-s13 * expD3));
      jacobianToPack.set(6, 5, exp2Alpha * (-expD3));
      jacobianToPack.set(6, 7, exp2Alpha * (-t3));
      jacobianToPack.set(6, 9, exp2Alpha * (-t1));
      // Row 7: Iyy = e^(2a)(s12^2 + s13^2 + t1^2 + t3^2 + e^(2 d1) + e^(2 d3))
      jacobianToPack.set(7, 0, 2.0 * exp2Alpha * (squareS12 + squareS13 + squareT1 + squareT3 + squareExpD1 + squareExpD3));
      jacobianToPack.set(7, 1, 2.0 * exp2Alpha * squareExpD1);
      jacobianToPack.set(7, 3, 2.0 * exp2Alpha * squareExpD3);
      jacobianToPack.set(7, 4, 2.0 * exp2Alpha * s12);
      jacobianToPack.set(7, 5, 2.0 * exp2Alpha * s13);
      jacobianToPack.set(7, 7, 2.0 * exp2Alpha * t1);
      jacobianToPack.set(7, 9, 2.0 * exp2Alpha * t3);
      // Row 8: Iyz = e^(2a)(-t2 t3 - s23 e^(d3))
      jacobianToPack.set(8, 0, 2.0 * exp2Alpha * (-t2 * t3 - s23 * expD3));
      jacobianToPack.set(8, 3, exp2Alpha * (-s23 * expD3));
      jacobianToPack.set(8, 6, exp2Alpha * (-expD3));
      jacobianToPack.set(8, 8, exp2Alpha * (-t3));
      jacobianToPack.set(8, 9, exp2Alpha * (-t2));
      // Row 9: Izz = e^(2a)(s12^2 + s13^2 + s23^2 + t1^2 + t2^2 + e^(2 d1) + e^(2 d2))
      jacobianToPack.set(9, 0, 2.0 * exp2Alpha * (squareS12 + squareS13 + squareS23 + squareT1 + squareT2 + squareExpD1 + squareExpD2));
      jacobianToPack.set(9, 1, 2.0 * exp2Alpha * squareExpD1);
      jacobianToPack.set(9, 2, 2.0 * exp2Alpha * squareExpD2);
      jacobianToPack.set(9, 4, 2.0 * exp2Alpha * s12);
      jacobianToPack.set(9, 5, 2.0 * exp2Alpha * s13);
      jacobianToPack.set(9, 6, 2.0 * exp2Alpha * s23);
      jacobianToPack.set(9, 7, 2.0 * exp2Alpha * t1);
      jacobianToPack.set(9, 8, 2.0 * exp2Alpha * t2);
   }

   /**
    * Sets the parameter vector with new values given in Pi basis.
    * <p>
    * Sets the Theta basis representation as stale, but DOES NOT update it (see {@link #update}).
    * </p>
    *
    * @param parameterVectorPiBasis the updated parameter vector in Pi basis.
    */
   public void setParameterVectorPiBasis(DMatrixRMaj parameterVectorPiBasis)
   {
      this.parameterVectorPiBasis.set(parameterVectorPiBasis);
      isPiBasisUpToDate = true;
      isThetaBasisUpToDate = false;
   }

   /**
    * Sets the parameter vector with new values given in Pi basis from a {@link us.ihmc.mecano.spatial.SpatialInertia}.
    * <p>
    * Sets the Theta basis representation as stale, but DOES NOT update it (see {@link #update}).
    * </p>
    *
    * @param spatialInertia the spatial inertia to update the parameter vector with.
    */
   public void setParameterVectorPiBasis(SpatialInertiaReadOnly spatialInertia)
   {
      double mass = spatialInertia.getMass();
      parameterVectorPiBasis.set(0, 0, mass);
      parameterVectorPiBasis.set(1, 0, spatialInertia.getCenterOfMassOffset().getX() * mass);
      parameterVectorPiBasis.set(2, 0, spatialInertia.getCenterOfMassOffset().getY() * mass);
      parameterVectorPiBasis.set(3, 0, spatialInertia.getCenterOfMassOffset().getZ() * mass);
      parameterVectorPiBasis.set(4, 0, spatialInertia.getMomentOfInertia().getM00());  // Ixx
      parameterVectorPiBasis.set(5, 0, spatialInertia.getMomentOfInertia().getM01());  // Ixy
      parameterVectorPiBasis.set(6, 0, spatialInertia.getMomentOfInertia().getM02());  // Ixz
      parameterVectorPiBasis.set(7, 0, spatialInertia.getMomentOfInertia().getM11());  // Iyy
      parameterVectorPiBasis.set(8, 0, spatialInertia.getMomentOfInertia().getM12());  // Iyz
      parameterVectorPiBasis.set(9, 0, spatialInertia.getMomentOfInertia().getM22());  // Izz

      isPiBasisUpToDate = true;
      isThetaBasisUpToDate = false;
   }

   /**
    * Sets the parameter vector with new values given in Theta basis.
    * <p>
    * Sets the Pi basis representation as stale, but DOES NOT update it (see {@link #update}).
    * </p>
    *
    * @param parameterVectorThetaBasis the updated parameter vector in Theta basis.
    */
   public void setParameterVectorThetaBasis(DMatrixRMaj parameterVectorThetaBasis)
   {
      this.parameterVectorThetaBasis.set(parameterVectorThetaBasis);
      isThetaBasisUpToDate = true;
      isPiBasisUpToDate = false;
   }

   public void getParameterVectorPiBasis(DMatrixRMaj parameterVectorPiBasisToPack)
   {
      parameterVectorPiBasisToPack.set(parameterVectorPiBasis);
   }

   public DMatrixRMaj getParameterVectorPiBasis()
   {
      return parameterVectorPiBasis;
   }

   public void getParameterVectorThetaBasis(DMatrixRMaj parameterVectorThetaBasisToPack)
   {
      parameterVectorThetaBasisToPack.set(parameterVectorThetaBasis);
   }

   public DMatrixRMaj getParameterVectorThetaBasis()
   {
      return parameterVectorThetaBasis;
   }

   public boolean isPiBasisUpToDate()
   {
      return isPiBasisUpToDate;
   }

   public boolean isThetaBasisUpToDate()
   {
      return isThetaBasisUpToDate;
   }

   public void reset()
   {
      // First reset everything
      parameterVectorPiBasis.zero();
      parameterVectorThetaBasis.zero();
      isPiBasisUpToDate = false;
      isThetaBasisUpToDate = false;

      // Second, make the pi basis be a spatial inertia with unit mass, slight CoM offset (to avoid numerical difficulties) and small spherical inertia
      // In the absence of a better guess, probably the best you can do
      parameterVectorPiBasis.set(0, 0, 1.0);
      parameterVectorPiBasis.set(1, 0, ZERO_COM_OFFSET_EPS);
      parameterVectorPiBasis.set(2, 0, ZERO_COM_OFFSET_EPS);
      parameterVectorPiBasis.set(3, 0, ZERO_COM_OFFSET_EPS);
      parameterVectorPiBasis.set(4, 0, 0.001);  // Ixx
      parameterVectorPiBasis.set(5, 0, 0.0);  // Ixy
      parameterVectorPiBasis.set(6, 0, 0.0);  // Ixz
      parameterVectorPiBasis.set(7, 0, 0.001);  // Iyy
      parameterVectorPiBasis.set(8, 0, 0.0);  // Iyz
      parameterVectorPiBasis.set(9, 0, 0.001);  // Izz
      isPiBasisUpToDate = true;
      update();
   }
}