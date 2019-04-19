package us.ihmc.robotDataLogger.logger;

@FunctionalInterface
public interface LogAliveListener
{
   void receivedLogAliveCommand(boolean camerasLogging);
}
