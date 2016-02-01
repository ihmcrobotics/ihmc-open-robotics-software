package us.ihmc.sensorProcessing.signalCorruption;

public interface SignalCorruptor<T>
{
   public abstract void corrupt(T signal);
}
