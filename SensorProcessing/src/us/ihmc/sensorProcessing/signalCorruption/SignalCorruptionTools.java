package us.ihmc.sensorProcessing.signalCorruption;

import java.util.ArrayList;

public class SignalCorruptionTools
{
   public static <T> void corrupt(T signal, ArrayList<SignalCorruptor<T>> corruptors)
   {
      for(int i = 0; i <  corruptors.size(); i++)
      {
         corruptors.get(i).corrupt(signal);
      }
   }
}
