package us.ihmc.parameterEstimation.inertial;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.SymmetricEigenDecomposition3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

import java.util.ArrayList;
import java.util.List;

public class RigidBodyInertialParametersTools
{
   /**
    * Checks if the input set of rigid body inertial parameters is physically consistent. Intended for use in debugging -- this method is not garbage-free. A
    * set of parameters is physically consistent if the following two conditions hold:
    *
    * <li>Mass is positive</li>
    * <li>Moment of inertia is positive definite</li>
    *
    * @param rigidBodyInertialParameters the set of rigidBodyInertialParameters to check for physical consistency.
    * @return true of the input set of rigid body inertial parameters are physically consistent.
    */
   public static boolean isPhysicallyConsistent(RigidBodyInertialParameters rigidBodyInertialParameters)
   {
      DMatrixRMaj parameters = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      rigidBodyInertialParameters.getParameterVectorPiBasis(parameters);

      double mass = parameters.get(0, 0);
      double Ixx = parameters.get(4, 0);
      double Ixy = parameters.get(5, 0);
      double Ixz = parameters.get(6, 0);
      double Iyy = parameters.get(7, 0);
      double Iyz = parameters.get(8, 0);
      double Izz = parameters.get(9, 0);

      return (mass > 0.0 && Matrix3DFeatures.isPositiveDefiniteMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz));
   }

   public static boolean isPhysicallyConsistent(SpatialInertiaReadOnly spatialInertia)
   {
      double mass = spatialInertia.getMass();
      double Ixx = spatialInertia.getMomentOfInertia().getM00();
      double Ixy = spatialInertia.getMomentOfInertia().getM01();
      double Ixz = spatialInertia.getMomentOfInertia().getM02();
      double Iyy = spatialInertia.getMomentOfInertia().getM11();
      double Iyz = spatialInertia.getMomentOfInertia().getM12();
      double Izz = spatialInertia.getMomentOfInertia().getM22();

      return (mass > 0.0 && Matrix3DFeatures.isPositiveDefiniteMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz));
   }

   /**
    * Checks if the input set of rigid body inertial parameters is fully physically consistent. Intended for use in debugging -- this method is not
    * garbage-free. A set of parameters is fully physically consistent if the following three conditions hold:
    *
    * <li>Mass is positive</li>
    * <li>Moment of inertia is positive definite</li>
    * <li>Principal moments of inertia satisfy a triangle inequality</li>
    *
    * @param rigidBodyInertialParameters the set of rigid body inertial parameters to check for full physical consistency.
    * @return true if the input set of rigid body inertial parameters are fully physically consistent.
    */
   public static boolean isFullyPhysicallyConsistent(RigidBodyInertialParameters rigidBodyInertialParameters)
   {
      DMatrixRMaj parameters = new DMatrixRMaj(RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1);
      rigidBodyInertialParameters.getParameterVectorPiBasis(parameters);

      double Ixx = parameters.get(4, 0);
      double Ixy = parameters.get(5, 0);
      double Ixz = parameters.get(6, 0);
      double Iyy = parameters.get(7, 0);
      double Iyz = parameters.get(8, 0);
      double Izz = parameters.get(9, 0);

      Matrix3D momentOfInertia = new Matrix3D(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

      SymmetricEigenDecomposition3D decomposition = new SymmetricEigenDecomposition3D();
      decomposition.decompose(momentOfInertia);

      Vector3D eigenvalues = decomposition.getEigenValues();
      boolean triangleInequalitySatisfied = eigenvalues.getX() < eigenvalues.getY() + eigenvalues.getZ() &&
                                            eigenvalues.getY() < eigenvalues.getX() + eigenvalues.getZ() &&
                                            eigenvalues.getZ() < eigenvalues.getX() + eigenvalues.getY();

      return (isPhysicallyConsistent(rigidBodyInertialParameters) && triangleInequalitySatisfied);
   }

   public static boolean isFullyPhysicallyConsistent(SpatialInertiaReadOnly spatialInertia)
   {
      Matrix3D momentOfInertia = new Matrix3D(spatialInertia.getMomentOfInertia());
      SymmetricEigenDecomposition3D decomposition = new SymmetricEigenDecomposition3D();
      decomposition.decompose(momentOfInertia);

      Vector3D eigenvalues = decomposition.getEigenValues();
      boolean triangleInequalitySatisfied = eigenvalues.getX() < eigenvalues.getY() + eigenvalues.getZ() &&
                                            eigenvalues.getY() < eigenvalues.getX() + eigenvalues.getZ() &&
                                            eigenvalues.getZ() < eigenvalues.getX() + eigenvalues.getY();

      return (isPhysicallyConsistent(spatialInertia) && triangleInequalitySatisfied);
   }

   /**
    * Calculates the difference between two sets of rigid body inertial parameters as represented by spatial inertias.
    *
    * @param inertia the input spatial inertia. Not modified.
    * @param referenceInertia the reference spatial inertia to calculate the parameter difference from. Not modified.
    * @param parameterDeltaToPack the parameter delta to store the result. Modified.
    */
   public static void calculateParameterDelta(SpatialInertiaReadOnly inertia, SpatialInertiaReadOnly referenceInertia, DMatrix parameterDeltaToPack)
   {
      // Mass
      double referenceMass = referenceInertia.getMass();
      double mass = inertia.getMass();
      parameterDeltaToPack.set(0, 0, mass - referenceMass);

      // First moment of mass
      double firstMomentOfMassDeltaX = (mass *  inertia.getCenterOfMassOffset().getX()) - (referenceMass * referenceInertia.getCenterOfMassOffset().getX());
      double firstMomentOfMassDeltaY = (mass *  inertia.getCenterOfMassOffset().getY()) - (referenceMass * referenceInertia.getCenterOfMassOffset().getY());
      double firstMomentOfMassDeltaZ = (mass *  inertia.getCenterOfMassOffset().getZ()) - (referenceMass * referenceInertia.getCenterOfMassOffset().getZ());
      parameterDeltaToPack.set(1, 0, firstMomentOfMassDeltaX);
      parameterDeltaToPack.set(2, 0, firstMomentOfMassDeltaY);
      parameterDeltaToPack.set(3, 0, firstMomentOfMassDeltaZ);

      // Unique elements of the moment of inertia
      parameterDeltaToPack.set(4, 0, inertia.getMomentOfInertia().getM00() - referenceInertia.getMomentOfInertia().getM00());  // Ixx
      parameterDeltaToPack.set(5, 0, inertia.getMomentOfInertia().getM01() - referenceInertia.getMomentOfInertia().getM01());  // Ixy
      parameterDeltaToPack.set(6, 0, inertia.getMomentOfInertia().getM02() - referenceInertia.getMomentOfInertia().getM02());  // Ixz
      parameterDeltaToPack.set(7, 0, inertia.getMomentOfInertia().getM11() - referenceInertia.getMomentOfInertia().getM11());  // Iyy
      parameterDeltaToPack.set(8, 0, inertia.getMomentOfInertia().getM12() - referenceInertia.getMomentOfInertia().getM12());  // Iyz
      parameterDeltaToPack.set(9, 0, inertia.getMomentOfInertia().getM22() - referenceInertia.getMomentOfInertia().getM22());  // Izz
   }

   /**
    * Adds on an inertial parameter delta to an input spatial inertia.
    *
    * @param inertia the input spatial inertia. Not modified.
    * @param parameterDelta the inertial parameter delta to add. Not modified.
    * @param resultToPack the spatial inertia to store the result. Modified.
    */
   public static void addParameterDelta(SpatialInertiaReadOnly inertia, DMatrix parameterDelta, SpatialInertiaBasics resultToPack)
   {
      // Mass
      double mass = inertia.getMass();
      double massDelta = parameterDelta.get(0, 0);
      resultToPack.setMass(mass + massDelta);

      // First moment of mass -- only need to update the center of mass offset here
      double firstMomentOfMassDeltaX = parameterDelta.get(1, 0);
      double firstMomentOfMassDeltaY = parameterDelta.get(2, 0);
      double firstMomentOfMassDeltaZ = parameterDelta.get(3, 0);
      resultToPack.setCenterOfMassOffset(inertia.getCenterOfMassOffset().getX() + firstMomentOfMassDeltaX / massDelta,
                                         inertia.getCenterOfMassOffset().getY() + firstMomentOfMassDeltaY / massDelta,
                                         inertia.getCenterOfMassOffset().getZ() + firstMomentOfMassDeltaZ / massDelta);

      // Unique elements of the moment of inertia
      resultToPack.setMomentOfInertia(inertia.getMomentOfInertia().getM00() + parameterDelta.get(4, 0),  // Ixx
                                      inertia.getMomentOfInertia().getM01() + parameterDelta.get(5, 0),  // Ixy
                                      inertia.getMomentOfInertia().getM02() + parameterDelta.get(6, 0),  // Ixz
                                      inertia.getMomentOfInertia().getM11() + parameterDelta.get(7, 0),  // Iyy
                                      inertia.getMomentOfInertia().getM12() + parameterDelta.get(8, 0),  // Iyz
                                      inertia.getMomentOfInertia().getM22() + parameterDelta.get(9, 0));  // Izz
   }

   /**
    * Zero a given inertial parameter in the spatial inertia of a rigid body.
    *
    * @param body the rigid body to zero the parameter of. Modified.
    * @param basis the basis representing the inertial parameter to be zeroed.
    */
   public static void zeroInertialParameter(RigidBodyBasics body, SpatialInertiaBasisOption basis)
   {
      SpatialInertiaBasics spatialInertia = body.getInertia();
      switch (basis)
      {
         case M -> spatialInertia.setMass(0.0);
         case MCOM_X -> spatialInertia.getCenterOfMassOffset().setX(0.0);
         case MCOM_Y -> spatialInertia.getCenterOfMassOffset().setY(0.0);
         case MCOM_Z -> spatialInertia.getCenterOfMassOffset().setZ(0.0);
         case I_XX -> spatialInertia.getMomentOfInertia().setM00(0.0);
         case I_XY ->
         {
            spatialInertia.getMomentOfInertia().setM01(0.0);
            spatialInertia.getMomentOfInertia().setM10(0.0);
         }
         case I_YY -> spatialInertia.getMomentOfInertia().setM11(0.0);
         case I_XZ ->
         {
            spatialInertia.getMomentOfInertia().setM02(0.0);
            spatialInertia.getMomentOfInertia().setM20(0.0);
         }
         case I_YZ ->
         {
            spatialInertia.getMomentOfInertia().setM12(0.0);
            spatialInertia.getMomentOfInertia().setM21(0.0);
         }
         case I_ZZ -> spatialInertia.getMomentOfInertia().setM22(0.0);
      }
   }

   public static String[] getNamesForPiBasis()
   {
      List<String> rowNames = new ArrayList<>();
      rowNames.add("M");
      rowNames.add("MCOMX");
      rowNames.add("MCOMY");
      rowNames.add("MCOMZ");
      rowNames.add("IXX");
      rowNames.add("IXY");
      rowNames.add("IXZ");
      rowNames.add("IYY");
      rowNames.add("IYZ");
      rowNames.add("IZZ");
      return rowNames.toArray(new String[0]);
   }

   public static String[] getNamesForThetaBasis()
   {
      List<String> rowNames = new ArrayList<>();
      rowNames.add("alpha");
      rowNames.add("d1");
      rowNames.add("d2");
      rowNames.add("d3");
      rowNames.add("s12");
      rowNames.add("s13");
      rowNames.add("s23");
      rowNames.add("t1");
      rowNames.add("t2");
      rowNames.add("t3");
      return rowNames.toArray(new String[0]);
   }
}