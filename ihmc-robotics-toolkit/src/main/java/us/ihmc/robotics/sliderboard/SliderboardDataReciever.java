package us.ihmc.robotics.sliderboard;

import java.util.ArrayList;
import java.util.List;

import javax.sound.midi.MidiMessage;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.procedure.TObjectProcedure;
import us.ihmc.commons.PrintTools;

public class SliderboardDataReciever implements Receiver
{
   private final MidiControlMap channelMapper;

   private final TIntObjectMap<List<SliderboardListener>> listeners = new TIntObjectHashMap<>();

   private final TIntDoubleMap values = new TIntDoubleHashMap();

   public SliderboardDataReciever(MidiControlMap channelMapper)
   {
      this.channelMapper = channelMapper;
   }

   @Override
   public void send(MidiMessage message, long timeStamp)
   {
      if (!(message instanceof ShortMessage))
      {
         return;
      }

      ShortMessage shortMessage = (ShortMessage) message;
      if (shortMessage.getCommand() != SliderboardTools.STATUS)
      {
         return;
      }

      int sliderChannel = shortMessage.getData1();
      int sliderIndex = channelMapper.getSliderIndex(sliderChannel);

      if (sliderIndex == -1)
      {
         PrintTools.info("Unknown controller: " + sliderChannel);
         return;
      }

      double value = SliderboardTools.toSliderPercent(shortMessage.getData2());

      List<SliderboardListener> sliderListeners = listeners.get(sliderIndex);
      double previousValue = values.containsKey(sliderIndex) ? values.get(sliderIndex) : -1.0;
      if (sliderListeners != null && previousValue != value)
      {
         sliderListeners.forEach(listener -> listener.sliderMoved(value));
      }
      values.put(sliderIndex, value);
   }

   public boolean addListener(SliderboardListener sliderListener, int sliderIndex)
   {
      if (channelMapper.getSliderChannel(sliderIndex) == -1)
      {
         return false;
      }

      if (!listeners.containsKey(sliderIndex))
      {
         listeners.put(sliderIndex, new ArrayList<>());
      }
      listeners.get(sliderIndex).add(sliderListener);
      return true;
   }

   @Override
   public void close()
   {
   }

   public void clearListeners()
   {
      listeners.forEachValue(new TObjectProcedure<List<SliderboardListener>>()
      {
         @Override
         public boolean execute(List<SliderboardListener> listenerList)
         {
            listenerList.clear();
            return true;
         }
      });
   }

   public void clearListeners(int sliderIndex)
   {
      if (listeners.containsKey(sliderIndex))
      {
         listeners.get(sliderIndex).clear();
      }
   }
}
