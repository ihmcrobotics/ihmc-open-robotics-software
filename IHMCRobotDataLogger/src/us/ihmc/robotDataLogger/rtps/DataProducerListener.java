package us.ihmc.robotDataLogger.rtps;

public interface DataProducerListener
{
   public void changeVariable(int id, double newValue);
}
