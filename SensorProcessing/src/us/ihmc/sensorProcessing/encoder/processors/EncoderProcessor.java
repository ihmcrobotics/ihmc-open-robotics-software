package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.sensorProcessing.ProcessedPositionSensor;
import us.ihmc.sensorProcessing.ProcessedVelocitySensor;


public interface EncoderProcessor extends ProcessedPositionSensor, ProcessedVelocitySensor
{
   /**
    * This needs to be called each computation tick, with the time between ticks approximately the same each tick.
    * Each EncoderProcessor should be constructed with the objects required to know the encoder raw state so that 
    * the processor can estimate the processed state. 
    */
   public abstract void update();

   /**
    * Sets the units that the position and velocity will be in, based on the unitDistancePerCount.
    * Should be called once at initialization and never again.
    */
   public abstract void setUnitDistancePerCount(double unitDistancePerCount);
}
