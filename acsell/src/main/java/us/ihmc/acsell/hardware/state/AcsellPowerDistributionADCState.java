package us.ihmc.acsell.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.yoVariables.variable.YoDouble;

public interface AcsellPowerDistributionADCState
{

   public void update(ByteBuffer buffer);
   
   public YoDouble getTotalWorkVariable();
   
}
