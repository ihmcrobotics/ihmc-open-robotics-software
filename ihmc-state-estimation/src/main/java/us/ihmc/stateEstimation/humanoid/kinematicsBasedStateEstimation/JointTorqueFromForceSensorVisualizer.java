package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueFromForceSensorVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<JacobianBasedWrenchEstimator> estimators = new ArrayList<>();

   public JointTorqueFromForceSensorVisualizer(RigidBodyBasics rootBody,
                                               Map<RigidBodyBasics, ? extends ContactablePlaneBody> feet,
                                               Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                               YoRegistry parentRegistry)
   {
      for (RigidBodyBasics foot : footSwitches.keySet())
      {
         estimators.add(new JacobianBasedWrenchEstimator(foot, rootBody, feet.get(foot).getSoleFrame(), footSwitches.get(foot), registry));
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (int i = 0; i < estimators.size(); i++)
      {
         estimators.get(i).update();
      }
   }

   private static class JacobianBasedWrenchEstimator
   {
      private final FootSwitchInterface footSwitch;

      private final GeometricJacobian jacobian;
      private final InverseDynamicsCalculator gravityTorqueCalculator;
      private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(6, 1);
      private final OneDoFJointReadOnly[] joints;
      private final YoDouble[] jointGravityTaus;
      private final YoDouble[] jointTausFromFTSensor;

      private final YoFixedFrameWrench wrench;
      private final YoFixedFrameWrench wrenchNoGravity;
      private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      private final DMatrixRMaj torqueVector = new DMatrixRMaj(6, 1);
      private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(6);

      public JacobianBasedWrenchEstimator(RigidBodyBasics foot,
                                          RigidBodyBasics rootBody,
                                          ReferenceFrame soleFrame,
                                          FootSwitchInterface footSwitch,
                                          YoRegistry registry)
      {
         this.footSwitch = footSwitch;

         joints = MultiBodySystemTools.createOneDoFJointPath(rootBody, foot);

         if (joints.length != 6)
            throw new RuntimeException("We can't yet use the Jacobian Based Wrench calculator, because the Jacobian isn't square. We need to implement this with a pseudo inverse.");

         jacobian = new GeometricJacobian(rootBody, foot, soleFrame);
         gravityTorqueCalculator = new InverseDynamicsCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(joints));
         gravityTorqueCalculator.setConsiderJointAccelerations(false);
         gravityTorqueCalculator.setGravitionalAcceleration(-9.81); // TODO Extract me

         jointGravityTaus = new YoDouble[6];
         jointTausFromFTSensor = new YoDouble[6];

         for (int i = 0; i < jointGravityTaus.length; i++)
         {
            jointGravityTaus[i] = new YoDouble("tau_gravity_" + joints[i].getName(), registry);
            jointTausFromFTSensor[i] = new YoDouble("tau_forceSensor_" + joints[i].getName(), registry);
         }
         String namePrefix = foot.getName() + "JTrans";

         wrench = new YoFixedFrameWrench(foot.getBodyFixedFrame(),
                                         new YoFrameVector3D(namePrefix + "EstimatedTorque", soleFrame, registry),
                                         new YoFrameVector3D(namePrefix + "EstimatedForce", soleFrame, registry));
         wrenchNoGravity = new YoFixedFrameWrench(foot.getBodyFixedFrame(),
                                                  new YoFrameVector3D(namePrefix + "EstimatedTorqueNoGravity", soleFrame, registry),
                                                  new YoFrameVector3D(namePrefix + "EstimatedForceNoGravity", soleFrame, registry));
      }

      private final Wrench forceTorqueSensorWrench = new Wrench();

      public void update()
      {
         jacobian.compute();
         jacobianTranspose.reshape(jacobian.getNumberOfColumns(), 6);
         CommonOps_DDRM.transpose(jacobian.getJacobianMatrix(), jacobianTranspose);

         for (int i = 0; i < joints.length; i++)
            torqueVector.set(i, 0, joints[i].getTau());

         solver.setA(jacobianTranspose);
         CommonOps_DDRM.scale(-1.0, torqueVector);

         solver.solve(torqueVector, wrenchVector);
         wrench.set(wrenchVector);

         gravityTorqueCalculator.compute();

         for (int i = 0; i < joints.length; i++)
         {
            jointGravityTaus[i].set(gravityTorqueCalculator.getComputedJointTau(joints[i]).get(0));
            torqueVector.set(i, 0, joints[i].getTau() - jointGravityTaus[i].getValue());
         }

         CommonOps_DDRM.scale(-1.0, torqueVector);

         solver.solve(torqueVector, wrenchVector);
         wrenchNoGravity.set(wrenchVector);

         forceTorqueSensorWrench.setIncludingFrame(footSwitch.getMeasuredWrench());
         forceTorqueSensorWrench.changeFrame(jacobian.getJacobianFrame());
         forceTorqueSensorWrench.negate();
         jacobian.computeJointTorques(forceTorqueSensorWrench, torqueVector);

         for (int i = 0; i < joints.length; i++)
         {
            jointTausFromFTSensor[i].set(torqueVector.get(i) + jointGravityTaus[i].getValue());
         }
      }
   }
}
