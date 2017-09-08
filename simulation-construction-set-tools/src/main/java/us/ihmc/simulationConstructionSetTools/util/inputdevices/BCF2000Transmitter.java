package us.ihmc.simulationConstructionSetTools.util.inputdevices;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

public class BCF2000Transmitter implements SliderBoardTransmitterInterface
{
   private MessageSender[] senderPool = new MessageSender[127];

   private Receiver midiOut;
   private static final boolean DEBUG = false;
   private int sliderBoardMax;
   private static final int SLIDER_OFFSET = 80;

   public BCF2000Transmitter(Receiver midiOut, int sliderBoardMax)
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
         if (senderPool[80 + midiControl.mapping] == null)
         {
            senderPool[80 + midiControl.mapping] = new MessageSender();
            new Thread(senderPool[80 + midiControl.mapping]).start();
         }

         return senderPool[80 + midiControl.mapping];
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
               shortMesssage.setMessage(176, 0, midiControl.mapping + SLIDER_OFFSET, valueToMoveTo);
               midiOut.send(shortMesssage, -1);
               Thread.sleep(100);
               if (moveBySliderValue)
                  valueToMoveTo = sliderValue;
               else
                  valueToMoveTo = SliderBoardUtils.valueRatioConvertToIntWithExponents(midiControl, sliderBoardMax);
               shortMesssage.setMessage(176, 0, midiControl.mapping + SLIDER_OFFSET, valueToMoveTo);
               midiOut.send(shortMesssage, -1);
            }
            catch (Exception e1)
            {
               e1.printStackTrace();
            }
         }
         
         printIfDebug("Gracefully exiting run method in " + getClass().getSimpleName());

      }
   }

   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }

   public class LightShow implements Runnable
   {
      @Override
      public void run()
      {
         int valueToMoveTo = 0;
         int tick = 0;
         int direction = 1;

         while (true)
         {
            try
            {
               Thread.sleep(100);
            }
            catch (InterruptedException e1)
            {
            }

            for (int i = 1; i < 9; i++)
            {
               if (i % 2 == 0)
               {
                  valueToMoveTo = tick;
               }
               else
                  valueToMoveTo = 127 - tick;

               ShortMessage shortMesssage = new ShortMessage();
               try
               {
                  shortMesssage.setMessage(176, 0, i, valueToMoveTo);
               }
               catch (InvalidMidiDataException e)
               {
                  e.printStackTrace();
               }

               midiOut.send(shortMesssage, -1);
               tick += direction;

               if (tick == 127)
               {
                  tick = 0;
               }

//             else if (tick == 0)
//             {
//               direction = 1;
//             }

            }
         }
      }

   }

}
