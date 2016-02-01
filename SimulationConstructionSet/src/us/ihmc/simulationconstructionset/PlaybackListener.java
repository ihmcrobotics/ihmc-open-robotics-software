package us.ihmc.simulationconstructionset;


public interface PlaybackListener extends IndexChangedListener
{
   public void play(double realTimeRate);

   public void stop();

   // public void setRealTimeRate(double realTimeRate);
}
