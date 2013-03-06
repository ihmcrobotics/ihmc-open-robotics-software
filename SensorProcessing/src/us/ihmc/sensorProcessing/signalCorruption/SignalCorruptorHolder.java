package us.ihmc.sensorProcessing.signalCorruption;

import java.util.ArrayList;
import java.util.List;

public class SignalCorruptorHolder<T>
{
   private final List<SignalCorruptor<T>> signalCorruptors = new ArrayList<SignalCorruptor<T>>();

   public final void addSignalCorruptor(SignalCorruptor<T> signalCorruptor)
   {
      signalCorruptors.add(signalCorruptor);
   }
   
   protected final void corrupt(T signal)
   {
      SignalCorruptionTools.corrupt(signal, signalCorruptors);
   }
}
