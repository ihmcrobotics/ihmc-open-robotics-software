package us.ihmc.gr00t;

import us.ihmc.sensors.ImageSensor;

/** Packs task-specific observations into a GR00T client's reusable wire buffers. */
public interface Gr00tObservationSource extends AutoCloseable
{
   boolean pack(ImageSensor imageSensor);

   @Override
   void close();
}
