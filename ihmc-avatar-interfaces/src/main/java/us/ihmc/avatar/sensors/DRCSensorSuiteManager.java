package us.ihmc.avatar.sensors;

import java.net.URI;

import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.tools.thread.CloseableAndDisposable;

public interface DRCSensorSuiteManager extends CloseableAndDisposable
{
   public void initializeSimulatedSensors(ObjectCommunicator localObjectCommunicator);

   public void initializePhysicalSensors(URI sensorURI);
   
   public void connect();
}
