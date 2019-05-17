package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;

/**
 * TODO: Split this class in two: The part that reads and the part that does the computation on the sensor data as they
 * might be called from seperate threads.
 */
public interface SensorReader
{
   /**
    * Reads the sensor data from the robot. This method should not do any heavy computation and is meant to populate the
    * sensor data context as quickly as possible to be used by the state estimator thread.
    * </p>
    * Called from a thread synchronized with the robot e.g. the EtherCAT thread.
    *
    * @param sensorDataContext is the sensor data object that this method should pack.
    * @return timestamp of sensor data.
    */
   public abstract long read(SensorDataContext sensorDataContext);

   /**
    * Called from the estimator thread. Does the computation required to update the sensor output map.
    * </p>
    * If running single threaded this might be called right after {@link SensorReader#read(SensorDataContext)}.
    *
    * @param timestamp of sensor data
    * @param sensorDataContext contains the raw sensor measurements.
    */
   public default void compute(long timestamp, SensorDataContext sensorDataContext)
   {
   };

   public abstract SensorOutputMapReadOnly getSensorOutputMapReadOnly();

   public abstract SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly();
}