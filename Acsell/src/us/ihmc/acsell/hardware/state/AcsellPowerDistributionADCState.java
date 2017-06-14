package us.ihmc.acsell.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.yoVariables.variable.DoubleYoVariable;

public interface AcsellPowerDistributionADCState
{

   public void update(ByteBuffer buffer);
   
   public DoubleYoVariable getTotalWorkVariable();
   
}
