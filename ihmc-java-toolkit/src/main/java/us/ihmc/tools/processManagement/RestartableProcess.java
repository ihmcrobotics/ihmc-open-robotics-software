package us.ihmc.tools.processManagement;

public interface RestartableProcess
{
   void start();

   void stop();

   String getName();
}
