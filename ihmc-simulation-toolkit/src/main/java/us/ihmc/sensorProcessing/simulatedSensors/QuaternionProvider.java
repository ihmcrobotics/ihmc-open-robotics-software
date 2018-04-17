package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface QuaternionProvider
{
   QuaternionReadOnly getValue();
}
