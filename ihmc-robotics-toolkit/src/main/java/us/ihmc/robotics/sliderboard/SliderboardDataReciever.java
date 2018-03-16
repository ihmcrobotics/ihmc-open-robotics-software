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
import us.ihmc.commons.PrintTools;

public class SliderboardDataReciever implements Receiver
{
   private final TIntObjectMap<List<SliderboardListener>> listeners = new TIntObjectHashMap<>();

   private final TIntDoubleMap values = new TIntDoubleHashMap();

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

      if (!SliderboardConfiguration.CONTROLLER_SLIDER_MAP.containsKey((byte) shortMessage.getData1()))
      {
         PrintTools.info("Unknown controller: " + shortMessage.getData1());
         return;
      }

      int slider = SliderboardConfiguration.CONTROLLER_SLIDER_MAP.get((byte) shortMessage.getData1());
      double value = SliderboardTools.toSliderPercent((byte) shortMessage.getData2());

      List<SliderboardListener> sliderListeners = listeners.get(slider);
      double previousValue = values.containsKey(slider) ? values.get(slider) : -1.0;
      if (sliderListeners != null && previousValue != value)
      {
         sliderListeners.forEach(listener -> listener.sliderMoved(value));
      }
      values.put(slider, value);
   }

   public void addListener(SliderboardListener sliderListener, int slider)
   {
      if (!SliderboardConfiguration.SLIDER_CONTROLLER_MAP.containsKey(slider))
      {
         PrintTools.info("Unknown slider index: " + slider);
         return;
      }
      if (!listeners.containsKey(slider))
      {
         listeners.put(slider, new ArrayList<>());
      }
      listeners.get(slider).add(sliderListener);
   }

   @Override
   public void close()
   {
   }
}
