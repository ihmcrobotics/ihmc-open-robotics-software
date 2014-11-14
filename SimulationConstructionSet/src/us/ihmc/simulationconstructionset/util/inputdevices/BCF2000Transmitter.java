package us.ihmc.simulationconstructionset.util.inputdevices;

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
//      Thread lightSHow = new Thread(new LightShow());
      //lightSHow.start();
      
   }
   
   public MessageSender getOrStartSender(MidiControl ctrl)
   {
      synchronized (senderPool)
      {
         if(senderPool[80 + ctrl.mapping] == null)
         {
            senderPool[80 + ctrl.mapping] = new MessageSender();
            new Thread(senderPool[80 + ctrl.mapping]).start();
         }
         return senderPool[80 + ctrl.mapping];
      }
   }

   public void moveControl(MidiControl ctrl)
   {
      if (midiOut != null)
      {
         if (DEBUG)
            System.out.println("ASetting control [" + ctrl.mapping + "] to " + ctrl.var.getValueAsDouble());
         getOrStartSender(ctrl).set(ctrl);

      }
   }

   public void moveControl(MidiControl ctrl, int sliderValue)
   {
      if (midiOut != null)
      {
         if (DEBUG)
            System.out.println("ASetting control [" + ctrl.mapping + "] to " + ctrl.var.getValueAsDouble());
         getOrStartSender(ctrl).set(ctrl, sliderValue);

      }
   }

   private class MessageSender implements Runnable
   {
      private Object lock = new Object();
      
      MidiControl ctrl;
      int sliderValue;
      boolean moveBySliderValue = false;


      public void set(MidiControl ctrl, int sliderValue)
      {
         synchronized (lock)
         {
            moveBySliderValue = true;
            this.sliderValue = sliderValue;
            this.ctrl = ctrl;
            lock.notify();
         }
      }

      public void set(MidiControl ctrl)
      {
         synchronized (lock)
         {
            this.ctrl = ctrl;
            lock.notify();
         }
      }

      public void run()
      {
         while(true)
         {
            MidiControl ctrl;
            int sliderValue;
            boolean moveBySliderValue;
            synchronized(lock)
            {
               while(this.ctrl == null)
               {
                  try
                  {
                     lock.wait();
                  }
                  catch (InterruptedException e)
                  {
                  }
               }
               ctrl = this.ctrl;
               sliderValue = this.sliderValue;
               moveBySliderValue = this.moveBySliderValue;
               
               this.ctrl = null;
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
                  valueToMoveTo = SliderBoardUtils.valueRatioConvertToIntWithExponents(ctrl, sliderBoardMax);
               if (valueToMoveTo < 0)
                  valueToMoveTo = 0;
               if (valueToMoveTo > 127)
                  valueToMoveTo = 127;
               shortMesssage.setMessage(176, 0, ctrl.mapping + SLIDER_OFFSET, valueToMoveTo);
               midiOut.send(shortMesssage, -1);
               Thread.sleep(100);
               if (moveBySliderValue)
                  valueToMoveTo = sliderValue;
               else
                  valueToMoveTo = SliderBoardUtils.valueRatioConvertToIntWithExponents(ctrl, sliderBoardMax);
               shortMesssage.setMessage(176, 0, ctrl.mapping + SLIDER_OFFSET, valueToMoveTo);
               midiOut.send(shortMesssage, -1);
            }
            catch (Exception e1)
            {
               e1.printStackTrace();
            }   
         }
         

      }

   }


   public class LightShow implements Runnable
   {
      public void run()
      {
         int valueToMoveTo = 0;
         int tick = 0;
         int direction = 1;

         while (true)
         {
        	 try {
				Thread.sleep(100);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
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
//               else if (tick == 0)
//               {
//            	   direction = 1;
//               }

            }
         }
      }

   }

}
