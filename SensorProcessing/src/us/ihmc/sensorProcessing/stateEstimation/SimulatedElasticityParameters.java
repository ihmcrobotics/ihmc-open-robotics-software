package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public interface SimulatedElasticityParameters
{
   public boolean isSpringJoint(OneDegreeOfFreedomJoint joint);
 
   public double getStiffness(OneDegreeOfFreedomJoint joint);

   public double getMaxDeflection(OneDegreeOfFreedomJoint simulatedJoint);
}
