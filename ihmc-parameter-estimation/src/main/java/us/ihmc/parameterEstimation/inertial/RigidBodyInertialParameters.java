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
    * This is taken from <a href="https://github.com/ebianchi/dair_pll/blob/e5908a1826ca6b9beaed8930887bf5e7a5d81e01/dair_pll/inertia.py#LL238C15-L238C15">LINK</a>
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
    * This is taken from <a href="https://github.com/ebianchi/dair_pll/blob/e5908a1826ca6b9beaed8930887bf5e7a5d81e01/dair_pll/inertia.py#LL238C15-L238C15">LINK</a>
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

      double expAlphaExpD1 = Math.sqrt(0.5 * (Iyy + Izz - Ixx));
      double expAlphaS12 = -Ixy / expAlphaExpD1;
      double expAlphaS13 = -Ixz / expAlphaExpD1;
      double expAlphaExpD2 = Math.sqrt(Izz - MathTools.pow(expAlphaExpD1, 2) - MathTools.pow(expAlphaS12, 2));
      double expAlphaS23 = (-Iyz - expAlphaS12 * expAlphaS13) / expAlphaExpD2;
      double expAlphaExpD3 = Math.sqrt(Iyy - MathTools.pow(expAlphaExpD1, 2) - MathTools.pow(expAlphaS13, 2) - MathTools.pow(expAlphaS23, 2));
      double expAlphaT1 = hx / expAlphaExpD1;
      double expAlphaT2 = (hy - expAlphaT1 * expAlphaS12) / expAlphaExpD2;
      double expAlphaT3 = (hz - expAlphaT1 * expAlphaS13 - expAlphaT2 * expAlphaS23) / expAlphaExpD3;
      double expAlpha = Math.sqrt(m - MathTools.pow(expAlphaT1, 2) - MathTools.pow(expAlphaT2, 2) - MathTools.pow(expAlphaT3, 2));

      parameterVectorThetaBasisToPack.set(0, Math.log(expAlpha));
      parameterVectorThetaBasisToPack.set(1, Math.log(expAlphaExpD1 / expAlpha));
      parameterVectorThetaBasisToPack.set(2, Math.log(expAlphaExpD2 / expAlpha));
      parameterVectorThetaBasisToPack.set(3, Math.log(expAlphaExpD3 / expAlpha));
      parameterVectorThetaBasisToPack.set(4, expAlphaS12 / expAlpha);
      parameterVectorThetaBasisToPack.set(5, expAlphaS13 / expAlpha);
      parameterVectorThetaBasisToPack.set(6, expAlphaS23 / expAlpha);
      parameterVectorThetaBasisToPack.set(7, expAlphaT1 / expAlpha);
      parameterVectorThetaBasisToPack.set(8, expAlphaT2 / expAlpha);
      parameterVectorThetaBasisToPack.set(9, expAlphaT3 / expAlpha);
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
      double squareS12 = MathTools.pow(s12, 2);
      double squareS13 = MathTools.pow(s13, 2);
      double squareS23 = MathTools.pow(s23, 2);

      parameterVectorPiBasisToPack.set(0, MathTools.pow(t1, 2) + MathTools.pow(t2, 2) + MathTools.pow(t3, 2) + 1);
      parameterVectorPiBasisToPack.set(1, t1 * expD1);
      parameterVectorPiBasisToPack.set(2, t1 * s12 + t2 * expD2);
      parameterVectorPiBasisToPack.set(3, t1 * s13 + t2 * s23 + t3 * expD3);
      parameterVectorPiBasisToPack.set(4, squareS12 + squareS13 + squareS23 + expD2 * expD2 + expD3 * expD3);
      parameterVectorPiBasisToPack.set(5, -s12 * expD1);
      parameterVectorPiBasisToPack.set(6, -s13 * expD1);
      parameterVectorPiBasisToPack.set(7, squareS13 + squareS23 + expD1 * expD1 + expD3 * expD3);
      parameterVectorPiBasisToPack.set(8, -s12 * s13 - s23 * expD2);
      parameterVectorPiBasisToPack.set(9, squareS12 + expD1 * expD1 + expD2 * expD2);
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
      double squareS12 = MathTools.pow(s12, 2);
      double squareS13 = MathTools.pow(s13, 2);
      double squareS23 = MathTools.pow(s23, 2);

      // First row, derivatives of mass with respect to theta
      jacobianToPack.set(0, 0, 2 * exp2Alpha * (MathTools.pow(t1, 2) + MathTools.pow(t2, 2) + MathTools.pow(t3, 2) + 1));
      jacobianToPack.set(0, 1, 0);
      jacobianToPack.set(0, 2, 0);
      jacobianToPack.set(0, 3, 0);
      jacobianToPack.set(0, 4, 0);
      jacobianToPack.set(0, 5, 0);
      jacobianToPack.set(0, 6, 0);
      jacobianToPack.set(0, 7, 2 * exp2Alpha * t1);
      jacobianToPack.set(0, 8, 2 * exp2Alpha * t2);
      jacobianToPack.set(0, 9, 2 * exp2Alpha * t3);
      // Second row, derivatives of hx with respect to theta
      jacobianToPack.set(1, 0, 2 * t1 * exp2Alpha * expD1);
      jacobianToPack.set(1, 1, t1 * exp2Alpha * expD1);
      jacobianToPack.set(1, 2, 0);
      jacobianToPack.set(1, 3, 0);
      jacobianToPack.set(1, 4, 0);
      jacobianToPack.set(1, 5, 0);
      jacobianToPack.set(1, 6, 0);
      jacobianToPack.set(1, 7, exp2Alpha * expD1);
      jacobianToPack.set(1, 8, 0);
      jacobianToPack.set(1, 9, 0);
      // Third row, derivatives of hy with respect to theta
      jacobianToPack.set(2, 0, 2 * exp2Alpha * t1 * s12 + t2 * expD2);
      jacobianToPack.set(2, 1, 0);
      jacobianToPack.set(2, 2, t2 * exp2Alpha * expD2);
      jacobianToPack.set(2, 3, 0);
      jacobianToPack.set(2, 4, t1 * exp2Alpha);
      jacobianToPack.set(2, 5, 0);
      jacobianToPack.set(2, 6, 0);
      jacobianToPack.set(2, 7, s12 * exp2Alpha);
      jacobianToPack.set(2, 8, exp2Alpha * expD2);
      jacobianToPack.set(2, 9, 0);
      // Fourth row, derivatives of hz with respect to theta
      jacobianToPack.set(3, 0, 2 * exp2Alpha * t1 * s13 + t2 * s23 + t3 * expD3);
      jacobianToPack.set(3, 1, 0);
      jacobianToPack.set(3, 2, 0);
      jacobianToPack.set(3, 3, t3 * exp2Alpha * expD3);
      jacobianToPack.set(3, 4, 0);
      jacobianToPack.set(3, 5, t1 * exp2Alpha);
      jacobianToPack.set(3, 6, t2 * exp2Alpha);
      jacobianToPack.set(3, 7, s13 * exp2Alpha);
      jacobianToPack.set(3, 8, s23 * exp2Alpha);
      jacobianToPack.set(3, 9, exp2Alpha * expD3);
      // Fifth row, derivatives of Ixx with respect to theta
      jacobianToPack.set(4, 0, 2 * exp2Alpha * (squareS12 + squareS13 + squareS23 + expD2 * expD2 + expD3 * expD3));
      jacobianToPack.set(4, 1, 0);
      jacobianToPack.set(4, 2, 2 * exp2Alpha * expD2 * expD2);
      jacobianToPack.set(4, 3, 2 * exp2Alpha * expD3 * expD3);
      jacobianToPack.set(4, 4, 2 * exp2Alpha * s12);
      jacobianToPack.set(4, 5, 2 * exp2Alpha * s13);
      jacobianToPack.set(4, 6, 2 * exp2Alpha * s23);
      jacobianToPack.set(4, 7, 0);
      jacobianToPack.set(4, 8, 0);
      jacobianToPack.set(4, 9, 0);
      // Sixth row, derivatives of Ixy with respect to theta
      jacobianToPack.set(5, 0, -2 * s12 * exp2Alpha * expD1);
      jacobianToPack.set(5, 1, -s12 * exp2Alpha * expD1);
      jacobianToPack.set(5, 2, 0);
      jacobianToPack.set(5, 3, 0);
      jacobianToPack.set(5, 4, -exp2Alpha * expD1);
      jacobianToPack.set(5, 5, 0);
      jacobianToPack.set(5, 6, 0);
      jacobianToPack.set(5, 7, 0);
      jacobianToPack.set(5, 8, 0);
      jacobianToPack.set(5, 9, 0);
      // Seventh row, derivatives of Ixz with respect to theta
      jacobianToPack.set(6, 0, -2 * s13 * exp2Alpha * expD1);
      jacobianToPack.set(6, 1, -s13 * exp2Alpha * expD1);
      jacobianToPack.set(6, 2, 0);
      jacobianToPack.set(6, 3, 0);
      jacobianToPack.set(6, 4, 0);
      jacobianToPack.set(6, 5, -exp2Alpha * expD1);
      jacobianToPack.set(6, 6, 0);
      jacobianToPack.set(6, 7, 0);
      jacobianToPack.set(6, 8, 0);
      jacobianToPack.set(6, 9, 0);
      // Eighth row, derivatives of Iyy with respect to theta
      jacobianToPack.set(7, 0, 2 * exp2Alpha * (squareS13 + squareS23 + expD1 * expD1 + expD3 * expD3));
      jacobianToPack.set(7, 1, 2 * exp2Alpha * expD1 * expD1);
      jacobianToPack.set(7, 2, 0);
      jacobianToPack.set(7, 3, 2 * exp2Alpha * expD3 * expD3);
      jacobianToPack.set(7, 4, 0);
      jacobianToPack.set(7, 5, 2 * exp2Alpha * s13);
      jacobianToPack.set(7, 6, 2 * exp2Alpha * s23);
      jacobianToPack.set(7, 7, 0);
      jacobianToPack.set(7, 8, 0);
      jacobianToPack.set(7, 9, 0);
      // Ninth row, derivatives of Iyz with respect to theta
      jacobianToPack.set(8, 0, 2 * exp2Alpha * (-s12 * s13 - s23 * expD2));
      jacobianToPack.set(8, 1, 0);
      jacobianToPack.set(8, 2, -s23 * exp2Alpha * expD2);
      jacobianToPack.set(8, 3, 0);
      jacobianToPack.set(8, 4, -s13 * exp2Alpha);
      jacobianToPack.set(8, 5, -s12 * exp2Alpha);
      jacobianToPack.set(8, 6, -exp2Alpha * expD2);
      jacobianToPack.set(8, 7, 0);
      jacobianToPack.set(8, 8, 0);
      jacobianToPack.set(8, 9, 0);
      // Tenth row, derivatives of Izz with respect to theta
      jacobianToPack.set(9, 0, 2 * exp2Alpha * (squareS12 + expD1 * expD1 + expD2 * expD2));
      jacobianToPack.set(9, 1, 2 * exp2Alpha * expD1 * expD1);
      jacobianToPack.set(9, 2, 2 * exp2Alpha * expD2 * expD2);
      jacobianToPack.set(9, 3, 0);
      jacobianToPack.set(9, 4, 2 * exp2Alpha * s12);
      jacobianToPack.set(9, 5, 0);
      jacobianToPack.set(9, 6, 0);
      jacobianToPack.set(9, 7, 0);
      jacobianToPack.set(9, 8, 0);
      jacobianToPack.set(9, 9, 0);
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