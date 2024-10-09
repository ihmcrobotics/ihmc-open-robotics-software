package us.ihmc.parameterTuner.sliderboard;

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

   private final TIntObjectMap<List<SliderboardListener>> sliderListeners = new TIntObjectHashMap<>();
   private final TIntObjectMap<List<ButtonListener>> buttonListeners = new TIntObjectHashMap<>();

   private final TIntDoubleMap sliderValues = new TIntDoubleHashMap();

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

      int midiChannel = shortMessage.getData1();
      if (channelMapper.getSliderIndex(midiChannel) != MidiControlMap.INVALID)
      {
         handleSliderInput(channelMapper.getSliderIndex(midiChannel), shortMessage.getData2());
      }
      else if (channelMapper.getButtonIndex(midiChannel) != MidiControlMap.INVALID)
      {
         handleButtonInput(channelMapper.getButtonIndex(midiChannel), shortMessage.getData2());
      }
      else if (channelMapper.getDelayVariationChannel() == midiChannel)
      {
         PrintTools.info("Got a 'Delay/Variation Send' message.");
      }
      else
      {
         PrintTools.info("Unknown controller: " + midiChannel + " - " + shortMessage.getData2());
         return;
      }
   }

   private void handleSliderInput(int sliderIndex, int data)
   {
      double value = SliderboardTools.toSliderPercent(data);

      List<SliderboardListener> listeners = sliderListeners.get(sliderIndex);
      double previousValue = sliderValues.containsKey(sliderIndex) ? sliderValues.get(sliderIndex) : -1.0;
      if (listeners != null && previousValue != value)
      {
         listeners.forEach(listener -> listener.sliderMoved(value));
      }
      sliderValues.put(sliderIndex, value);
   }

   private void handleButtonInput(int buttonIndex, int data)
   {
      boolean status = SliderboardTools.toButtonStatus(data);

      List<ButtonListener> listeners = buttonListeners.get(buttonIndex);
      if (listeners != null)
      {
         listeners.forEach(listener -> listener.buttonPressed(status));
      }
   }

   public boolean addListener(SliderboardListener sliderListener, int sliderIndex)
   {
      if (channelMapper.getSliderChannel(sliderIndex) == MidiControlMap.INVALID)
      {
         return false;
      }

      if (!sliderListeners.containsKey(sliderIndex))
      {
         sliderListeners.put(sliderIndex, new ArrayList<>());
      }
      sliderListeners.get(sliderIndex).add(sliderListener);
      return true;
   }

   public boolean addListener(ButtonListener buttonListener, int buttonIndex)
   {
      if (channelMapper.getButtonChannel(buttonIndex) == MidiControlMap.INVALID)
      {
         return false;
      }

      if (!buttonListeners.containsKey(buttonIndex))
      {
         buttonListeners.put(buttonIndex, new ArrayList<>());
      }
      buttonListeners.get(buttonIndex).add(buttonListener);
      return true;
   }

   public void clearListeners()
   {
      sliderListeners.forEachValue(new TObjectProcedure<List<SliderboardListener>>()
      {
         @Override
         public boolean execute(List<SliderboardListener> listenerList)
         {
            listenerList.clear();
            return true;
         }
      });

      buttonListeners.forEachValue(new TObjectProcedure<List<ButtonListener>>()
      {
         @Override
         public boolean execute(List<ButtonListener> listenerList)
         {
            listenerList.clear();
            return true;
         }
      });
   }

   public void removeListener(SliderboardListener sliderListener, int sliderIndex)
   {
      if (sliderListeners.containsKey(sliderIndex))
      {
         sliderListeners.get(sliderIndex).remove(sliderListener);
      }
   }

   public void removeListener(ButtonListener buttonListener, int buttonIndex)
   {
      if (buttonListeners.containsKey(buttonIndex))
      {
         buttonListeners.get(buttonIndex).remove(buttonListener);
      }
   }

   @Override
   public void close()
   {
   }
}
