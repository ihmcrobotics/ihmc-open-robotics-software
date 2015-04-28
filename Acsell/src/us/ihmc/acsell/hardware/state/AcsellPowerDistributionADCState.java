package us.ihmc.acsell.hardware.state;

import java.nio.ByteBuffer;

public interface AcsellPowerDistributionADCState
{

   public void update(ByteBuffer buffer);
   
}
