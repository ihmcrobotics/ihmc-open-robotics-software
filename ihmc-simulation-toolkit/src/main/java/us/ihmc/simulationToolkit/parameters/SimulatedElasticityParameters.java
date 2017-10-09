package us.ihmc.simulationToolkit.parameters;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public interface SimulatedElasticityParameters
{
   public boolean isSpringJoint(OneDegreeOfFreedomJoint joint);
 
   public double getStiffness(OneDegreeOfFreedomJoint joint);

   public double getMaxDeflection(OneDegreeOfFreedomJoint simulatedJoint);
}
