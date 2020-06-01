package us.ihmc.robotics.screwTheory;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

public interface KinematicLoopFunction
{
   DMatrixRMaj getLoopJacobian();

   DMatrixRMaj getLoopConvectiveTerm();

   void computeTau(DMatrixRMaj tauJoints);

   List<? extends OneDoFJointReadOnly> getLoopJoints();
}
