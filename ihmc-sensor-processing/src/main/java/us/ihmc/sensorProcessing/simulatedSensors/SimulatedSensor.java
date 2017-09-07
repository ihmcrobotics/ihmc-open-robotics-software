package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptionTools;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptor;

public abstract class SimulatedSensor<T> extends AbstractControlFlowElement
{
   private final ArrayList<SignalCorruptor<T>> signalCorruptors = new ArrayList<SignalCorruptor<T>>();
   
   public final void addSignalCorruptor(SignalCorruptor<T> signalCorruptor)
   {
      signalCorruptors.add(signalCorruptor);
   }

   protected final void corrupt(T signal)
   {
      SignalCorruptionTools.corrupt(signal, signalCorruptors);
   }
}
