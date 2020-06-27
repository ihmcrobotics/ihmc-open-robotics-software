package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public interface MassMatrixCalculator
{

   public abstract void compute();

   public abstract DMatrixRMaj getMassMatrix();

   public abstract void getMassMatrix(DMatrixRMaj massMatrixToPack);

   public abstract JointBasics[] getJointsInOrder();
}