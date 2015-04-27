package us.ihmc.acsell.hardware;

import us.ihmc.acsell.parameters.StrainGaugeInformation;

public interface AcsellJoint
{
   public String getSdfName();
   
   public double getRatio();
   
   public AcsellActuator[] getActuators();
   
   public boolean isLinear();
   
   public boolean hasOutputEncoder();
   
   public StrainGaugeInformation getStrainGaugeInformation();
}
