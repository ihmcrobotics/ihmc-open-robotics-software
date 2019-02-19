package us.ihmc.robotDataLogger.interfaces;

public interface RegistryPublisher
{

   void start();

   void stop();

   void update(long timestamp);

}