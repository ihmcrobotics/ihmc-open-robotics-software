package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.scs2.definition.controller.interfaces.Controller;

public interface ExternalTorqueEstimatorInterface extends Controller
{
   DMatrixRMaj getEstimatedExternalTorque();

   void requestInitialize();

   void setEstimatorGain(double estimatorGain);
}
