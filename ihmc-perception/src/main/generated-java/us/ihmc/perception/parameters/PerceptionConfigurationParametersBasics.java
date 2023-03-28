package us.ihmc.perception.parameters;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PerceptionConfigurationParametersBasics extends PerceptionConfigurationParametersReadOnly, StoredPropertySetBasics
{
   default void setL515ThrottlerFrequency(int l515ThrottlerFrequency)
   {
      set(PerceptionConfigurationParameters.l515ThrottlerFrequency, l515ThrottlerFrequency);
   }

   default void setOusterThrottlerFrequency(int ousterThrottlerFrequency)
   {
      set(PerceptionConfigurationParameters.ousterThrottlerFrequency, ousterThrottlerFrequency);
   }

   default void setRapidRegionsEnabled(boolean rapidRegionsEnabled)
   {
      set(PerceptionConfigurationParameters.rapidRegionsEnabled, rapidRegionsEnabled);
   }

   default void setLoggingEnabled(boolean loggingEnabled)
   {
      set(PerceptionConfigurationParameters.loggingEnabled, loggingEnabled);
   }

   default void setPublishColor(boolean publishColor)
   {
      set(PerceptionConfigurationParameters.publishColor, publishColor);
   }

   default void setPublishDepth(boolean publishDepth)
   {
      set(PerceptionConfigurationParameters.publishDepth, publishDepth);
   }

   default void setLogColor(boolean logColor)
   {
      set(PerceptionConfigurationParameters.logColor, logColor);
   }

   default void setLogDepth(boolean logDepth)
   {
      set(PerceptionConfigurationParameters.logDepth, logDepth);
   }

   default void setSLAMEnabled(boolean slamEnabled)
   {
      set(PerceptionConfigurationParameters.slamEnabled, slamEnabled);
   }

   default void setSLAMReset(boolean slamReset)
   {
      set(PerceptionConfigurationParameters.slamReset, slamReset);
   }
}
