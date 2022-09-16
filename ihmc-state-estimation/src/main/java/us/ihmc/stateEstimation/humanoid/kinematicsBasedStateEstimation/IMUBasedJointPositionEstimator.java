package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.function.ToDoubleFunction;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import cern.colt.Arrays;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.DampedLeastSquaresNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class IMUBasedJointPositionEstimator
{
   private final GeometricJacobianCalculator jacobianCalculator;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   private final YoDouble[] jointPositionCorrection;
   private final OneDoFJointBasics[] joints;

   private final YoDouble errorGain;
   private final YoDouble preferredGain;

   private final DMatrixRMaj jacobianAngularPart;
   private final DMatrixRMaj primaryJacobian;
   private final DMatrixRMaj errorEJML = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj errorEJMLSubspace = new DMatrixRMaj(3, 1);
   private final DampedLeastSquaresSolver solver;

//   private RotationErrorCalculator errorCalculator = new SelectionMatrixBasedCalculatorB();
      private RotationErrorCalculator errorCalculator = new VectorComparisonBasedCalculator();

   private final Matrix3D selectionMatrix3D = new Matrix3D();
   private final DMatrixRMaj selectionMatrixEJML = new DMatrixRMaj(3, 3);
   private final Quaternion childOrientationKinematics = new Quaternion();
   private final Quaternion childOrientationInParentKinematics = new Quaternion();

   private final DMatrixRMaj qCorrection;

   private ToDoubleFunction<OneDoFJointBasics> preferredJointPositions;
   private final DMatrixRMaj qPreferred;
   private final NullspaceCalculator nullspaceCalculator;
   private final DMatrixRMaj preferredJacobian;

   private final DMatrixRMaj solverInput_A = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj solverInput_b = new DMatrixRMaj(3, 1);

   public IMUBasedJointPositionEstimator(String namePrefix, IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoRegistry registry)
   {
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;
      
      joints = MultiBodySystemTools.createOneDoFJointPath(parentIMU.getMeasurementLink(), childIMU.getMeasurementLink());
      this.jacobianCalculator = new GeometricJacobianCalculator();
      jacobianCalculator.setKinematicChain(joints);
      jacobianCalculator.setJacobianFrame(childIMU.getMeasurementFrame());

      errorGain = new YoDouble(namePrefix + "ErrorGain", registry);
      errorGain.set(0.9);
      preferredGain = new YoDouble(namePrefix + "PreferredGain", registry);
      preferredGain.set(0.45);
      jointPositionCorrection = new YoDouble[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         jointPositionCorrection[i] = new YoDouble(namePrefix + "_q_corr_" + joint.getName(), registry);
      }

      solver = new DampedLeastSquaresSolver(joints.length);
      solver.setAlpha(1.0e-4);
      jacobianAngularPart = new DMatrixRMaj(3, joints.length);
      primaryJacobian = new DMatrixRMaj(3, joints.length);
      qCorrection = new DMatrixRMaj(joints.length, 1);
      qPreferred = new DMatrixRMaj(joints.length, 1);
      nullspaceCalculator = new DampedLeastSquaresNullspaceCalculator(joints.length, 1.0e-7);
      preferredJacobian = new DMatrixRMaj(joints.length, joints.length);
   }

   public void setPreferredJointPositions(ToDoubleFunction<OneDoFJointBasics> preferredJointPositions)
   {
      this.preferredJointPositions = preferredJointPositions;
   }

   public void compute()
   {
      ReferenceFrame parentIMUFrame = parentIMU.getMeasurementFrame();
      ReferenceFrame childIMUFrame = childIMU.getMeasurementFrame();

      jacobianCalculator.setJacobianFrame(childIMUFrame);
      jacobianCalculator.reset();
      // jacobian is 6xn
      CommonOps_DDRM.extract(jacobianCalculator.getJacobianMatrix(), 0, 3, 0, joints.length, jacobianAngularPart, 0, 0);

      QuaternionReadOnly parentOrientationIMU = parentIMU.getOrientationMeasurement();
      QuaternionReadOnly childOrientationIMU = childIMU.getOrientationMeasurement();
      childOrientationInParentKinematics.setToZero();
      childIMUFrame.transformFromThisToDesiredFrame(parentIMUFrame, childOrientationInParentKinematics);
      childOrientationKinematics.multiply(parentOrientationIMU, childOrientationInParentKinematics);
      Vector3DReadOnly error = errorCalculator.computeErrorRotationVector(parentOrientationIMU,
                                                                          childOrientationIMU,
                                                                          childOrientationInParentKinematics,
                                                                          childOrientationKinematics);
      error.get(errorEJML);
      CommonOps_DDRM.scale(errorGain.getValue(), errorEJML);

      selectionMatrix3D.setToDiagonal(1, 1, 0);
      childOrientationKinematics.inverseTransform(selectionMatrix3D);
      selectionMatrix3D.get(selectionMatrixEJML);
      CommonOps_DDRM.mult(selectionMatrixEJML, jacobianAngularPart, primaryJacobian);
      CommonOps_DDRM.mult(selectionMatrixEJML, errorEJML, errorEJMLSubspace);

      if (preferredJointPositions == null)
      {
         qPreferred.zero();
         solver.setA(primaryJacobian);
         solver.solve(errorEJMLSubspace, qCorrection);
      }
      else
      {
         for (int i = 0; i < joints.length; i++)
            qPreferred.set(i, 0, preferredJointPositions.applyAsDouble(joints[i]) - joints[i].getQ());
         CommonOps_DDRM.scale(preferredGain.getValue(), qPreferred);
         CommonOps_DDRM.setIdentity(preferredJacobian);
         nullspaceCalculator.projectOntoNullspace(preferredJacobian, primaryJacobian);

         solverInput_A.reshape(primaryJacobian.getNumRows() + preferredJacobian.getNumRows(), joints.length);
         CommonOps_DDRM.insert(primaryJacobian, solverInput_A, 0, 0);
         CommonOps_DDRM.insert(preferredJacobian, solverInput_A, primaryJacobian.getNumRows(), 0);
         solverInput_b.reshape(errorEJMLSubspace.getNumRows() + qPreferred.getNumRows(), 1);
         CommonOps_DDRM.insert(errorEJMLSubspace, solverInput_b, 0, 0);
         CommonOps_DDRM.insert(qPreferred, solverInput_b, errorEJMLSubspace.getNumRows(), 0);

         solver.setA(solverInput_A);
         solver.solve(solverInput_b, qCorrection);

//         System.out.println("error: " + error);
//         System.out.println("qPref : " + Arrays.toString(qPreferred.data));
//         System.out.println("qCorr : " + Arrays.toString(qCorrection.data));
      }

      for (int i = 0; i < joints.length; i++)
      {
         jointPositionCorrection[i].set(qCorrection.get(i));
      }
   }

   public YoDouble[] getJointPositionCorrection()
   {
      return jointPositionCorrection;
   }

   public DMatrixRMaj getQCorrection()
   {
      return qCorrection;
   }

   public DMatrixRMaj getQPreferred()
   {
      return qPreferred;
   }

   public DMatrixRMaj getPrimaryJacobian()
   {
      return primaryJacobian;
   }

   public DMatrixRMaj getPreferredJacobian()
   {
      return preferredJacobian;
   }

   interface RotationErrorCalculator
   {
      Vector3DReadOnly computeErrorRotationVector(QuaternionReadOnly parentOrientationIMU,
                                                  QuaternionReadOnly childOrientationIMU,
                                                  QuaternionReadOnly childOrientationInParentKinematics,
                                                  QuaternionReadOnly childOrientationKinematics);
   }

   static class SelectionMatrixBasedCalculatorA implements RotationErrorCalculator
   {
      private final Quaternion childOrientationInParentIMU = new Quaternion();
      private final Quaternion errorQuaternion = new Quaternion();
      private final Vector3D errorRotationVector = new Vector3D();
      private final Matrix3D selectionMatrix = new Matrix3D();

      @Override
      public Vector3DReadOnly computeErrorRotationVector(QuaternionReadOnly parentOrientationIMU,
                                                         QuaternionReadOnly childOrientationIMU,
                                                         QuaternionReadOnly childOrientationInParentKinematics,
                                                         QuaternionReadOnly childOrientationKinematics)
      {
         childOrientationInParentIMU.difference(parentOrientationIMU, childOrientationIMU);
         errorQuaternion.difference(childOrientationInParentKinematics, childOrientationInParentIMU);
         errorQuaternion.getRotationVector(errorRotationVector);

         selectionMatrix.setToDiagonal(1, 1, 0);
         childOrientationKinematics.inverseTransform(selectionMatrix);
//         selectionMatrix.transform(errorRotationVector);

         return errorRotationVector;
      }
   }

   static class SelectionMatrixBasedCalculatorB implements RotationErrorCalculator
   {
      private final Quaternion errorQuaternion = new Quaternion();
      private final Vector3D errorRotationVector = new Vector3D();
      private final Matrix3D selectionMatrix = new Matrix3D();

      @Override
      public Vector3DReadOnly computeErrorRotationVector(QuaternionReadOnly parentOrientationIMU,
                                                         QuaternionReadOnly childOrientationIMU,
                                                         QuaternionReadOnly childOrientationInParentKinematics,
                                                         QuaternionReadOnly childOrientationKinematics)
      {
         // R^Ckin_Cimu = R^Ckin_W R^W_Cimu
         errorQuaternion.difference(childOrientationKinematics, childOrientationIMU);
         errorQuaternion.getRotationVector(errorRotationVector);

         selectionMatrix.setToDiagonal(1, 1, 0);
         childOrientationKinematics.inverseTransform(selectionMatrix);
//         selectionMatrix.transform(errorRotationVector);

         return errorRotationVector;
      }
   }

   static class VectorComparisonBasedCalculator implements RotationErrorCalculator
   {
      private final Quaternion errorQuaternion = new Quaternion();
      private final Vector3D errorRotationVector = new Vector3D();

      private final Vector3D zUpIMU = new Vector3D();
      private final Vector3D zUpKinematics = new Vector3D();

      @Override
      public Vector3DReadOnly computeErrorRotationVector(QuaternionReadOnly parentOrientationIMU,
                                                         QuaternionReadOnly childOrientationIMU,
                                                         QuaternionReadOnly childOrientationInParentKinematics,
                                                         QuaternionReadOnly childOrientationKinematics)
      {
         childOrientationIMU.inverseTransform(Axis3D.Z, zUpIMU);
         childOrientationKinematics.inverseTransform(Axis3D.Z, zUpKinematics);
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(zUpIMU, zUpKinematics, errorQuaternion);
         errorQuaternion.getRotationVector(errorRotationVector);
         return errorRotationVector;
      }
   }
}
