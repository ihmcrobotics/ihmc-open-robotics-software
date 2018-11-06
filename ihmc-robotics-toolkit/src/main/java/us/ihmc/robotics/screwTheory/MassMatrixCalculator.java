package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public interface MassMatrixCalculator
{

   public abstract void compute();

   public abstract DenseMatrix64F getMassMatrix();

   public abstract void getMassMatrix(DenseMatrix64F massMatrixToPack);

   public abstract JointBasics[] getJointsInOrder();
}