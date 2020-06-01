package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

public interface KinematicLoopControllerFunction
{
   DMatrixRMaj getLoopJacobian();

   DMatrixRMaj getLoopConvectiveTerm();

   void computeTau(DMatrixRMaj tauJoints);

   List<? extends OneDoFJointReadOnly> getLoopJoints();
}
