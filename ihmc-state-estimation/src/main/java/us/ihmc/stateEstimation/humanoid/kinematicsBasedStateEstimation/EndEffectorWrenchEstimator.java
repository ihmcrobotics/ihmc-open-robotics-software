package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class EndEffectorWrenchEstimator
{
   private final String name;
   private final YoFixedFrameWrench wrench;
   private final JointTorqueProvider jointTorqueProvider;

   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final DenseMatrix64F jointTorqueMatrix;
   private final DenseMatrix64F jacobianTranspose;
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(6, 1);
   private final OneDoFJointBasics[] joints;

   private final LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.general(6, 6);

   public EndEffectorWrenchEstimator(String prefix, RigidBodyBasics base, RigidBodyBasics endEffector, ReferenceFrame wrenchMeasurementFrame,
                                     YoVariableRegistry registry)
   {
      this(prefix, base, endEffector, wrenchMeasurementFrame, OneDoFJointReadOnly::getTau, registry);
   }

   public EndEffectorWrenchEstimator(String prefix, RigidBodyBasics base, RigidBodyBasics endEffector, ReferenceFrame wrenchMeasurementFrame,
                                     JointTorqueProvider jointTorqueProvider, YoVariableRegistry registry)
   {
      this.jointTorqueProvider = jointTorqueProvider;

      name = prefix + getClass().getSimpleName();
      wrench = new YoFixedFrameWrench(name, endEffector.getBodyFixedFrame(), wrenchMeasurementFrame, registry);

      joints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);

      geometricJacobianCalculator.setKinematicChain(base, endEffector);
      geometricJacobianCalculator.setJacobianFrame(wrenchMeasurementFrame);

      if (joints.length != geometricJacobianCalculator.getNumberOfDegreesOfFreedom())
         throw new RuntimeException("This calculator only supports OneDoFJoints.");

      int dofs = geometricJacobianCalculator.getNumberOfDegreesOfFreedom();

      if (dofs != Wrench.SIZE)
         throw new RuntimeException("This calculator only supports square Jacobian matrix for now.");

      jointTorqueMatrix = new DenseMatrix64F(dofs, 1);
      jacobianTranspose = new DenseMatrix64F(dofs, Wrench.SIZE);
   }

   public void calculate()
   {
      geometricJacobianCalculator.reset();

      for (int i = 0; i < joints.length; i++)
         jointTorqueMatrix.set(i, 0, -jointTorqueProvider.getJointTorque(joints[i]));

      CommonOps.transpose(geometricJacobianCalculator.getJacobianMatrix(), jacobianTranspose);

      linearSolver.setA(jacobianTranspose);
      linearSolver.solve(jointTorqueMatrix, wrenchMatrix);

      wrench.set(wrenchMatrix);
   }

   public WrenchReadOnly getWrench()
   {
      return wrench;
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
