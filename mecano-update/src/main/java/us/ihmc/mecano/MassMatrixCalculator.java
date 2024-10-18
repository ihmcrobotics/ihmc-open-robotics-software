package us.ihmc.mecano;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public interface MassMatrixCalculator
{

   void compute();

   DMatrixRMaj getMassMatrix();

   void getMassMatrix(DMatrixRMaj massMatrixToPack);

   JointBasics[] getJointsInOrder();
}