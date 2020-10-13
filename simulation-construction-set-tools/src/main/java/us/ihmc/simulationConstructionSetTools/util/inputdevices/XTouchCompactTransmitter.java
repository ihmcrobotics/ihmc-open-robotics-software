package us.ihmc.simulationConstructionSetTools.util.inputdevices;

import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiControl.ControlType;

public class XTouchCompactTransmitter implements SliderBoardTransmitterInterface
{
   private MessageSender[] senderPool = new MessageSender[256];

   private Receiver midiOut;
   private static final boolean DEBUG = false;
   private int sliderBoardMax;

   public XTouchCompactTransmitter(Receiver midiOut, int sliderBoardMax)
   {
      this.midiOut = midiOut;
      this.sliderBoardMax = sliderBoardMax;

//    Thread lightSHow = new Thread(new LightShow());
      // lightSHow.start();

   }

   public MessageSender getOrStartSender(MidiControl midiControl)
   {
      synchronized (senderPool)
      {
         if (senderPool[midiControl.mapping] == null)
         {
            senderPool[midiControl.mapping] = new MessageSender();
            new Thread(senderPool[midiControl.mapping], "XTouchCompactTransmitter").start();
         }

         return senderPool[midiControl.mapping];
      }
   }

   @Override
   public void moveControl(MidiControl midiControl)
   {
      if (midiOut != null)
      {
         printIfDebug("ASetting control [" + midiControl.mapping + "] to " + midiControl.var.getValueAsDouble());
         getOrStartSender(midiControl).set(midiControl);

      }
   }

   @Override
   public void moveControl(MidiControl midiControl, int sliderValue)
   {
      if (midiOut != null)
      {
         printIfDebug("ASetting control [" + midiControl.mapping + "] to " + midiControl.var.getValueAsDouble());
         getOrStartSender(midiControl).set(midiControl, sliderValue);

      }
   }
   
   @Override
   public void closeAndDispose()
   {
      printIfDebug("Closing and Disposing Message Senders!");
      for (MessageSender messageSender : senderPool)
      {
         if (messageSender != null) messageSender.closeAndDispose();
      }
   }

   private class MessageSender implements Runnable
   {
      private Object lock = new Object();

      private MidiControl midiControl;
      private int sliderValue;
      private boolean moveBySliderValue = false;

      public void set(MidiControl midiControl, int sliderValue)
      {
         synchronized (lock)
         {
            moveBySliderValue = true;
            this.sliderValue = sliderValue;
            this.midiControl = midiControl;
            lock.notify();
         }
      }

      public void set(MidiControl midiControl)
      {
         synchronized (lock)
         {
            this.midiControl = midiControl;
            lock.notify();
         }
      }

      private boolean isRunning = true;
      
      public void closeAndDispose()
      {
         isRunning = false;
         
         synchronized(lock)
         {
            lock.notifyAll();
         }
      }
      
      @Override
      public void run()
      {
         while (isRunning)
         {
            MidiControl midiControl;
            int sliderValue;
            boolean moveBySliderValue;

            synchronized (lock)
            {
               while ((isRunning) && (this.midiControl == null))
               {
                  try
                  {
                     lock.wait();
                  }
                  catch (InterruptedException e)
                  {
                  }
               }

               if (!isRunning) return;
               
               midiControl = this.midiControl;
               sliderValue = this.sliderValue;
               moveBySliderValue = this.moveBySliderValue;

               this.midiControl = null;
               this.sliderValue = -1;
               this.moveBySliderValue = false;
            }

            if(midiControl.controlType == ControlType.KNOB || midiControl.controlType == ControlType.SLIDER)
            {
               ShortMessage shortMesssage = new ShortMessage();
               try
               {
                  int valueToMoveTo = 0;
   
                  if (moveBySliderValue)
                     valueToMoveTo = sliderValue;
                  else
                     valueToMoveTo = SliderBoardUtils.valueRatioConvertToIntWithExponents(midiControl, sliderBoardMax);
                  if (valueToMoveTo < 0)
                     valueToMoveTo = 0;
                  if (valueToMoveTo > 127)
                     valueToMoveTo = 127;
                  shortMesssage.setMessage(176, 0, midiControl.mapping, valueToMoveTo);
                  midiOut.send(shortMesssage, -1);
                  Thread.sleep(100);
                  if (moveBySliderValue)
                     valueToMoveTo = sliderValue;
                  else
                     valueToMoveTo = SliderBoardUtils.valueRatioConvertToIntWithExponents(midiControl, sliderBoardMax);
                  shortMesssage.setMessage(176, 0, midiControl.mapping, valueToMoveTo);
                  midiOut.send(shortMesssage, -1);
               }
               catch (Exception e1)
               {
                  e1.printStackTrace();
               }
            }
            else
            {
               ShortMessage shortMesssage = new ShortMessage();
               try
               {
                  boolean buttonValue = midiControl.var.getValueAsDouble() > 0.5;
                  printIfDebug("Setting " + midiControl.var + " to " + buttonValue);
                  shortMesssage.setMessage(144, 0, midiControl.mapping - 127, buttonValue ? 127 : 0);
                  midiOut.send(shortMesssage, -1);
               }
               catch (Exception e1)
               {
                  e1.printStackTrace();
               }
            }
               
         }
         
         printIfDebug("Gracefully exiting run method in " + getClass().getSimpleName());

      }
   }

   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }


}
