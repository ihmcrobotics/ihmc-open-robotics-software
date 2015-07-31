package us.ihmc.simulationconstructionset.simulatedSensors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface WrenchCalculatorInterface
{
   public abstract String getName();

   public abstract void getTransformToParentJoint(RigidBodyTransform transformToPack);

   public abstract void calculate();

   public abstract DenseMatrix64F getWrench();

   public abstract OneDegreeOfFreedomJoint getJoint();

   public abstract void corruptWrenchElement(int row, double value);

   public abstract void setDoWrenchCorruption(boolean value);

}