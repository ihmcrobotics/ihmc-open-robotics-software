package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Collection;

public class SignalCorruptionTools
{
   public static <T> void corrupt(T signal, Collection<SignalCorruptor<T>> corruptors)
   {
      for (SignalCorruptor<T> signalCorruptor : corruptors)
      {
         signalCorruptor.corrupt(signal);
      }
   }
}
