package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.simulationconstructionset.util.RobotController;

public interface ExternalTorqueEstimatorInterface extends RobotController
{
   DMatrixRMaj getEstimatedExternalTorque();

   void requestInitialize();

   void setEstimatorGain(double estimatorGain);

   double getEstimatedExternalTorqueMagnitude();
}
