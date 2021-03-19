package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.simulationconstructionset.util.RobotController;

/**
 * Given the manipulator equation: M*qdd + C*qd + G = tau + tau_ext.
 *
 * Where tau_ext is torque due to unmodelled external contact.
 * This interface is a solver for tau_ext given the state of the system over time.
 */
public interface ExternalTorqueEstimator extends RobotController
{
   DMatrixRMaj getObservedExternalJointTorque();

   void setEstimatorGain(double gain);
}
