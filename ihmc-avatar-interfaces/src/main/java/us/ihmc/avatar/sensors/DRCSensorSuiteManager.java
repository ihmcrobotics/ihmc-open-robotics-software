package us.ihmc.avatar.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.communication.net.ObjectCommunicator;

public interface DRCSensorSuiteManager
{
   public void initializeSimulatedSensors(ObjectCommunicator localObjectCommunicator);

   public void initializePhysicalSensors(URI sensorURI);
   
   public void connect() throws IOException;

}
