package us.ihmc.perception.parameters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.parameters.PerceptionConfigurationParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PerceptionConfigurationParametersReadOnly extends StoredPropertySetReadOnly
{
   default int getL515ThrottlerFrequency()
   {
      return get(l515ThrottlerFrequency);
   }

   default int getOusterThrottlerFrequency()
   {
      return get(ousterThrottlerFrequency);
   }

   default boolean getLoggingEnabled()
   {
      return get(loggingEnabled);
   }

   default boolean getPublishColor()
   {
      return get(publishColor);
   }

   default boolean getPublishDepth()
   {
      return get(publishDepth);
   }

   default boolean getLogColor()
   {
      return get(logColor);
   }

   default boolean getLogDepth()
   {
      return get(logDepth);
   }
}