package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.dataBuffer.IndexChangedListener;

public interface PlaybackListener extends IndexChangedListener
{
   public void play(double realTimeRate);

   public void stop();

   // public void setRealTimeRate(double realTimeRate);
}
