package us.ihmc.robotDataLogger.interfaces;

import java.io.IOException;

import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.dataBuffers.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public interface DataProducer
{

   /**
    * Deactivate the data producer. 
    * 
    * After calling this function, the producer cannot be reactivated
    */
   void remove();

   /**
    * Set the handshake data
    * 
    * Required
    * 
    * @param handshake
    */
   void setHandshake(Handshake handshake);

   /** 
    * Add cameras to log 
    * 
    * Optional
    * 
    * @param name User friendly name to show in the log files
    * @param cameraId ID of the camera on the logger machine
    */
   void addCamera(CameraType type, String name, String cameraId);

   /**
    * Activate the producer. This will publish the model, handshake and logger announcement to the logger
    * 
    * @throws IOException
    */
   void announce() throws IOException;

   /**
    * Set if the logger should record this session or not
    * 
    * @param log
    */
   void setLog(boolean log);

   /** 
    * Publisher a timestamp update
    * @param timestamp
    * @throws IOException 
    */
   void publishTimestamp(long timestamp);

   RegistryPublisher createRegistryPublisher(CustomLogDataPublisherType type, PeriodicThreadSchedulerFactory schedulerFactory,
                                             RegistrySendBufferBuilder builder)
         throws IOException;

   void sendKeepAlive(PeriodicThreadSchedulerFactory schedulerFactory) throws IOException;

}