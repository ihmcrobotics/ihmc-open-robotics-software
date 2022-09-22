package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.function.ToDoubleFunction;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
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
   private static final boolean USE_NULLSPACE_PROJECTOR = false;

   private boolean usePreferredPositions = false;

   private final GeometricJacobianCalculator jacobianCalculator;
   private final IMUSensorReadOnly parentIMU;
   private final IMUSensorReadOnly childIMU;
   private final YoDouble[] jointPositionCorrection;
   private final OneDoFJointBasics[] joints;

   private final YoDouble regularizationWeight;
   private final YoDouble errorGain;
   private final YoDouble[] errorWeight;
   private final YoDouble[] preferredWeight;

   private final DMatrixRMaj jacobianAngularPart;
   private final DMatrixRMaj primaryJacobian;
   private final DMatrixRMaj errorEJML = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj errorEJMLSubspace = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj primaryWeightMatrix;
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
   private final DMatrixRMaj preferredWeightMatrix;

   private final DMatrixRMaj solverInput_H = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj solverInput_f = new DMatrixRMaj(3, 1);

   public IMUBasedJointPositionEstimator(String namePrefix, IMUSensorReadOnly parentIMU, IMUSensorReadOnly childIMU, YoRegistry registry)
   {
      this.parentIMU = parentIMU;
      this.childIMU = childIMU;

      joints = MultiBodySystemTools.createOneDoFJointPath(parentIMU.getMeasurementLink(), childIMU.getMeasurementLink());
      this.jacobianCalculator = new GeometricJacobianCalculator();
      jacobianCalculator.setKinematicChain(joints);
      jacobianCalculator.setJacobianFrame(childIMU.getMeasurementFrame());

      regularizationWeight = new YoDouble(namePrefix + "RegularizationWeight", registry);
      regularizationWeight.set(1.0e-6);
      errorGain = new YoDouble(namePrefix + "ErrorGain", registry);
      errorGain.set(1);
      errorWeight = new YoDouble[3];
      //errorWeight.set(1);
      preferredWeight = new YoDouble[joints.length];
      //preferredWeight.set(0.01);
      jointPositionCorrection = new YoDouble[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         jointPositionCorrection[i] = new YoDouble(namePrefix + "_q_corr_" + joint.getName(), registry);
         preferredWeight[i] = new YoDouble(namePrefix + "PreferredWeight" + joint.getName(), registry);
         preferredWeight[i].set(0.01);
      }
      for (int i = 0; i < 3; i++)
      {
         errorWeight[i] = new YoDouble(namePrefix + "ErrorWeight" + Axis3D.values[i].name(), registry);
         errorWeight[i].set(1.0);
      }

      solver = new DampedLeastSquaresSolver(joints.length);
      jacobianAngularPart = new DMatrixRMaj(3, joints.length);
      primaryJacobian = new DMatrixRMaj(3, joints.length);
      qCorrection = new DMatrixRMaj(joints.length, 1);
      qPreferred = new DMatrixRMaj(joints.length, 1);
      primaryWeightMatrix = new DMatrixRMaj(3, 3);
      preferredWeightMatrix = new DMatrixRMaj(joints.length, joints.length);
      nullspaceCalculator = USE_NULLSPACE_PROJECTOR ? new DampedLeastSquaresNullspaceCalculator(joints.length, 1.0e-5) : null;
      preferredJacobian = new DMatrixRMaj(joints.length, joints.length);
   }

   public void setPreferredJointPositions(ToDoubleFunction<OneDoFJointBasics> preferredJointPositions)
   {
      this.preferredJointPositions = preferredJointPositions;
      usePreferredPositions = true;
   }


   public void setQPreferredPerJoint(OneDoFJointBasics joint, double desiredWeight)
   {
      usePreferredPositions = true;
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].equals(joint))
         {
            qPreferred.set(i, desiredWeight);
            break;
         }
      }
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

      if (usePreferredPositions == false)
      {
         qPreferred.zero();
         solver.setAlpha(regularizationWeight.getValue());
         solver.setA(primaryJacobian);
         solver.solve(errorEJMLSubspace, qCorrection);
      }
      else
      {
         // In this scenario, we want to solve the problem as an unconstrained QP so we can weigh primary vs preferred objectives. 
         if (preferredJointPositions != null)
         {
            for (int i = 0; i < joints.length; i++)
               qPreferred.set(i, 0, preferredJointPositions.applyAsDouble(joints[i]) - joints[i].getQ());
         }
         CommonOps_DDRM.setIdentity(preferredJacobian);
         if (USE_NULLSPACE_PROJECTOR)
            nullspaceCalculator.projectOntoNullspace(preferredJacobian, primaryJacobian);

         solverInput_H.reshape(joints.length, joints.length);
         CommonOps_DDRM.setIdentity(solverInput_H);
         // Squaring the regularizationWeight to be consistent with the DLS solver
         CommonOps_DDRM.scale(MathTools.square(regularizationWeight.getValue()), solverInput_H);
         solverInput_f.reshape(joints.length, 1);
         solverInput_f.zero();

         primaryWeightMatrix.zero();
         for (int i = 0; i < 3; i++)
         {
            primaryWeightMatrix.set(i, i, errorWeight[i].getDoubleValue());
         }

         preferredWeightMatrix.zero();
         for (int i = 0; i < joints.length; i++)
         {
            preferredWeightMatrix.set(i, i, preferredWeight[i].getDoubleValue());
         }

         addObjective(primaryWeightMatrix, primaryJacobian, errorEJMLSubspace, solverInput_H, solverInput_f);
         addObjective(preferredWeightMatrix, preferredJacobian, qPreferred, solverInput_H, solverInput_f);

         // min(qDot) 0.5 qDot' * H * qDot + f' * qDot
         // => solve H * qDot = -f
         CommonOps_DDRM.scale(-1, solverInput_f);

         solver.setAlpha(0.0);
         solver.setA(solverInput_H);
         solver.solve(solverInput_f, qCorrection);
      }

      for (int i = 0; i < joints.length; i++)
      {
         jointPositionCorrection[i].set(qCorrection.get(i));
      }
   }

   private final DMatrixRMaj temp_Jt = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp_JtW = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp_H = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj temp_f = new DMatrixRMaj(3, 1);

   private void addObjective(DMatrixRMaj weight, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj H, DMatrixRMaj f)
   {
      // We're to to do: min_qDot (J qDot - b)^T W (J qDot - b)
      // Which becomes: min_qDot 0.5 qDot^T J^T W J qDot - b^T W J qDot = min_qDot 0.5 qDot^T H qDot + f qDot
      // With: H = J^T W J, & f = -b^T W J

      temp_Jt.reshape(taskJacobian.getNumCols(), taskJacobian.getNumRows());
      CommonOps_DDRM.transpose(taskJacobian, temp_Jt);
      CommonOps_DDRM.mult(temp_Jt, weight, temp_JtW);

      // Compute: H += J^T W J
      CommonOps_DDRM.mult(temp_JtW, taskJacobian, temp_H);
      CommonOps_DDRM.addEquals(H, temp_H);

      // Compute: f += - b^T W J 
      CommonOps_DDRM.mult(temp_JtW, taskObjective, temp_f);
      CommonOps_DDRM.subtractEquals(f, temp_f);
   }

   public YoDouble[] getJointPositionCorrection()
   {
      return jointPositionCorrection;
   }

   public YoDouble getJointPositionCorrectionPerJoint(OneDoFJointBasics joint)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].equals(joint))
         {
            return jointPositionCorrection[i];
         }
      }

      return null;
   }

   public DMatrixRMaj getQCorrection()
   {
      return qCorrection;
   }

   public double getQCorrectionPerJoint(OneDoFJointBasics joint)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].equals(joint))
         {
            return qCorrection.get(i);
         }
      }

      return 0;
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

   public DMatrixRMaj getPreferredWeightMatrix()
   {
      return preferredWeightMatrix;
   }

   public void setPreferredQWeightPerJoint(OneDoFJointBasics joint, double desiredWeight)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].equals(joint))
         {
            preferredWeight[i].set(desiredWeight);
            break;
         }
      }
   }

   public void setPrimaryQWeightPerAxis(Axis3D axis, double desiredWeight)
   {
      preferredWeight[axis.ordinal()].set(desiredWeight);
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
         selectionMatrix.transform(errorRotationVector);

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
         selectionMatrix.transform(errorRotationVector);

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
