package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class EndEffectorWrenchEstimator
{
   private final String name;
   private final YoDouble jacobianDeterminant;
   private final YoFixedFrameWrench appliedWrench;
   private final WrenchReadOnly externalWrench;
   private final JointTorqueProvider jointTorqueProvider;

   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final DMatrixRMaj jointTorqueMatrix;
   private final DMatrixRMaj jacobianTranspose;
   private final DMatrixRMaj wrenchMatrix = new DMatrixRMaj(6, 1);
   private final OneDoFJointBasics[] joints;

   public EndEffectorWrenchEstimator(String prefix, RigidBodyBasics base, RigidBodyBasics endEffector, ReferenceFrame wrenchMeasurementFrame,
                                     YoRegistry registry)
   {
      this(prefix, base, endEffector, wrenchMeasurementFrame, OneDoFJointReadOnly::getTau, registry);
   }

   public EndEffectorWrenchEstimator(String name, RigidBodyBasics base, RigidBodyBasics endEffector, ReferenceFrame wrenchMeasurementFrame,
                                     JointTorqueProvider jointTorqueProvider, YoRegistry registry)
   {
      this.name = name;
      this.jointTorqueProvider = jointTorqueProvider;

      appliedWrench = new YoFixedFrameWrench(name, endEffector.getBodyFixedFrame(), wrenchMeasurementFrame, registry);
      externalWrench = MecanoFactories.newWrenchReadOnly(() -> -1.0, appliedWrench);
      jacobianDeterminant = new YoDouble(name + "JacobianDet", registry);

      joints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);

      geometricJacobianCalculator.setKinematicChain(base, endEffector);
      geometricJacobianCalculator.setJacobianFrame(wrenchMeasurementFrame);

      if (joints.length != geometricJacobianCalculator.getNumberOfDegreesOfFreedom())
         throw new RuntimeException("This calculator only supports OneDoFJoints.");

      int dofs = geometricJacobianCalculator.getNumberOfDegreesOfFreedom();

      if (dofs != Wrench.SIZE)
         throw new RuntimeException("This calculator only supports square Jacobian matrix for now.");

      jointTorqueMatrix = new DMatrixRMaj(dofs, 1);
      jacobianTranspose = new DMatrixRMaj(dofs, Wrench.SIZE);
   }

   public void calculate()
   {
      geometricJacobianCalculator.reset();

      for (int i = 0; i < joints.length; i++)
         jointTorqueMatrix.set(i, 0, jointTorqueProvider.getJointTorque(joints[i]));

      CommonOps_DDRM.transpose(geometricJacobianCalculator.getJacobianMatrix(), jacobianTranspose);
      jacobianDeterminant.set(CommonOps_DDRM.det(jacobianTranspose));
      NativeCommonOps.solveCheck(jacobianTranspose, jointTorqueMatrix, wrenchMatrix);

      appliedWrench.set(wrenchMatrix);
   }

   public double getJacobianDeterminant()
   {
      return jacobianDeterminant.getValue();
   }

   public WrenchReadOnly getAppliedWrench()
   {
      return appliedWrench;
   }

   public WrenchReadOnly getExternalWrench()
   {
      return externalWrench;
   }

   public String getName()
   {
      return name;
   }

   public static interface JointTorqueProvider
   {
      double getJointTorque(OneDoFJointBasics joint);
   }
}
