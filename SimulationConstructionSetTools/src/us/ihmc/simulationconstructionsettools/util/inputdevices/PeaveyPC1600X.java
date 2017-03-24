package us.ihmc.simulationconstructionsettools.util.inputdevices;

import java.util.ArrayList;

import javax.sound.midi.MetaMessage;
import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.SysexMessage;
import javax.sound.midi.Transmitter;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.ExitActionListener;


public class PeaveyPC1600X implements Receiver, ExitActionListener
{
   private static final boolean printInfo = false;

   private YoVariable<?>[] variables = new YoVariable[16];
   private String[] names = new String[16];
   private YoVariableRegistry holder = null;

   private double[] minVals = new double[16];
   private double[] maxVals = new double[16];
   private double[] exponents = new double[16];

   private MidiDevice inDevice;

   private void findPotentialDeviceNumbers(MidiDevice.Info[] infos, ArrayList<Integer> inDeviceNumbers, ArrayList<Integer> outDeviceNumbers)
   {
      if (printInfo)
         System.out.println("infos length: " + infos.length);

      for (int i = 0; i < infos.length; i++)
      {
         MidiDevice.Info info = infos[i];

         if (printInfo)
         {
            System.out.println("Device " + i + " of " + infos.length);
            System.out.println("Vendor: " + info.getVendor());
            System.out.println("Name: " + info.getName());
            System.out.println("Version: " + info.getVersion());

            System.out.println("Description: " + info.getDescription());
            System.out.println();
         }

         if (info.getName().indexOf("YAMAHA USB IN") >= 0)
         {
            if (printInfo)
               System.out.println("Adding Device Number " + i + " to list of potential input devices.\n");
            inDeviceNumbers.add(new Integer(i));
         }

         else if (info.getName().indexOf("YAMAHA USB OUT") >= 0)
         {
            if (printInfo)
               System.out.println("Adding Device Number " + i + " to list of potential output devices.");
            outDeviceNumbers.add(new Integer(i));
         }

      }

      if (printInfo)
      {
         System.out.println("Found " + inDeviceNumbers.size() + " potential Peavey PC1600X input devices");
         System.out.println("     and " + inDeviceNumbers.size() + " potential Peavey PC1600X output devices");
      }
   }

   public PeaveyPC1600X()
   {
      MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();

      ArrayList<Integer> inDeviceNumbers = new ArrayList<Integer>(), outDeviceNumbers = new ArrayList<Integer>();
      findPotentialDeviceNumbers(infos, inDeviceNumbers, outDeviceNumbers);

      if (inDeviceNumbers.isEmpty())
      {
         System.err.println("Couldn't find a Yamaha USB In device. Peavey PC1600X currently requires the Yamaha UX16");

         return;
      }

      if (inDeviceNumbers.size() > 1)
         System.err.println("Found multiple Yamaha USB In Devices. Going to try the first one found");
      int inDeviceNumber = inDeviceNumbers.get(0).intValue();
      MidiDevice.Info inInfo = infos[inDeviceNumber];
      if (printInfo)
         System.out.println("\nGetting device number " + inDeviceNumber + " with info: " + inInfo);

      try
      {
         inDevice = MidiSystem.getMidiDevice(inInfo);

         if (printInfo)
         {
            System.out.println("\nGot device with info" + inDevice.getDeviceInfo());
            System.out.println("Max Receivers:" + inDevice.getMaxReceivers());
            System.out.println("Max Transmitters:" + inDevice.getMaxTransmitters());
         }

         if (inDevice.isOpen())
         {
            if (printInfo)
               System.out.println("\nDevice started open. Closing Device\n");
            inDevice.close();

            if (inDevice.isOpen())
            {
               System.err.println("\nMIDI device started open. Attempted to close it, but could not. You may need to reboot to close the device.");
            }
         }

         else
         {
            if (printInfo)
               System.out.println("Device started closed");
         }

         if (printInfo)
            System.out.println("\nOpening Device\n");

         try
         {
            inDevice.open();
         }
         catch (Exception e)
         {
            System.out.println("Exception while trying to open device: " + e);
         }

         if (!inDevice.isOpen())
         {
            System.err.println("Cannot open MIDI device!");

            return;
         }

         if (printInfo)
            System.out.println("Device is Now open");
         if (printInfo)
            System.out.println("Getting Transmitter\n");

         try
         {
            Transmitter transmitter = inDevice.getTransmitter();
            transmitter.setReceiver(this);
         }
         catch (Exception e)
         {
            System.err.println("Exception when trying to get MIDI transmitter: " + e);
         }

         /*
          *  The following would be used if we could request that all the channels be sent initially.
          * //However, I couldn't find how to do that, so for now, hit the enter key on the 1600X to send all
          * //the slider values.
          * try
          * {
          * outDevice = MidiSystem.getMidiDevice(outInfo);
          *
          * System.out.println("\nGetting Receiver\n");
          * Receiver receiver = outDevice.getReceiver();
          *
          * MidiMessage message = new SysexMessage();
          * //message.
          *
          * long timeStamp = System.currentTimeMillis();
          * receiver.send(message, timeStamp);
          * }
          * catch (Exception e)
          * {
          * System.err.println("Exception when trying to get MIDI receiver: " + e);
          * }
          */
      }

      catch (Exception e)
      {
         System.out.println("Exception: " + e);
      }

   }

   public void setChannel(int channel, String name, YoVariableRegistry holder, double min, double max)
   {
      setChannel(channel, name, holder, min, max, 1.0);
   }

   public void setChannel(int channel, String name, YoVariableRegistry holder, double min, double max, double exponent)
   {
      if ((channel < 1) || (channel > 16))
      {
         System.err.println("Peavey PC1600X: Channel out of range.  Needs to be between 1 and 16");

         return;
      }

      this.holder = holder;

      variables[channel - 1] = holder.getVariable(name);
      names[channel - 1] = name;

      minVals[channel - 1] = min;
      maxVals[channel - 1] = max;
      exponents[channel - 1] = exponent;

   }

   public void setChannel(int channel, DoubleYoVariable var, double min, double max)
   {
      setChannel(channel, var, min, max, 1.0);
   }

   public void setChannel(int channel, DoubleYoVariable var, double min, double max, double exponent)
   {
      if ((channel < 1) || (channel > 16))
      {
         System.err.println("Peavey PC1600X: Channel out of range.  Needs to be between 1 and 16");

         return;
      }

      if (exponent <= 0.0)
      {
         System.err.println("Peavey PC1600X: Exponent must be positive. Setting it to 1.0");
         exponent = 1.0;
      }

      variables[channel - 1] = var;
      if (var != null)
         names[channel - 1] = var.getName();

      minVals[channel - 1] = min;
      maxVals[channel - 1] = max;
      exponents[channel - 1] = exponent;
   }

   public void setRange(int channel, double min, double max)
   {
      setRange(channel, min, max, 1.0);
   }

   public void setRange(int channel, double min, double max, double exponent)
   {
      if ((channel < 1) || (channel > 16))
      {
         System.err.println("Channel out of range.  Needs to be between 1 and 16");

         return;
      }

      if (exponent <= 0.0)
      {
         System.err.println("Peavey PC1600X: Exponent must be positive. Setting it to 1.0");
         exponent = 1.0;
      }

      minVals[channel - 1] = min;
      maxVals[channel - 1] = max;
      exponents[channel - 1] = exponent;

   }

   public static void main(String[] args)
   {
      PeaveyPC1600X peavey = new PeaveyPC1600X();

      long startTime = System.currentTimeMillis();

      while (System.currentTimeMillis() < startTime + 1000 * 20)
      {
         // System.out.println(device.getMicrosecondPosition());
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }
      }

      peavey.close();
      System.out.println("Device is now closed");
      System.exit(1);
   }

   @Override
   public void close()
   {
      System.out.println("Closing Receiver");

      if (inDevice != null)
         inDevice.close();
   }

   @Override
   public void send(MidiMessage message, long timeStamp)
   {
      if (message instanceof ShortMessage)
      {
         decodeMessage((ShortMessage) message);
      }
      else if (message instanceof SysexMessage)
      {
         decodeMessage((SysexMessage) message);
      }
      else if (message instanceof MetaMessage)
      {
         decodeMessage((MetaMessage) message);
      }
      else
      {
         // Unknown message type
      }
   }

   public void decodeMessage(ShortMessage message)
   {
      int channel = message.getChannel();
      int data2 = message.getData2();

      // System.out.println("ShortMessage");
      // System.out.println("Channel: " + channel);
      // System.out.println("Command: " + command);
      // System.out.println("Data 1: " + data1);
      // System.out.println("Data 2: " + data2);
      // System.out.println();

      if ((channel >= 0) && (channel < 16))
      {
         sliderSlid(channel, data2);
      }
   }

   public void decodeMessage(SysexMessage message)
   {
      System.out.println("SysexMessage");
   }

   public void decodeMessage(MetaMessage message)
   {
      System.out.println("MetaMessage");
   }

   public void sliderSlid(int channel, int sliderVal)
   {
	   YoVariable<?> variable = variables[channel];
      if (variable == null)
      {
         if (holder != null)
            variables[channel] = variable = holder.getVariable(names[channel]);
         if (variable == null)
            return;
      }

      // channel is between 0 and 15.  value is between 0 and 127.

      double max = maxVals[channel];
      double min = minVals[channel];
      double exponent = exponents[channel];

      double alpha = 2.0 * (sliderVal) / 127.0 - 1.0;
      double beta = Math.pow(Math.abs(alpha), exponent);
      if (alpha < 0.0)
         beta = -beta;

      double value = (min + max) / 2.0 + (max - min) / 2.0 * beta;

      variable.setValueFromDouble(value);

      // System.out.println(channel + ": " + contents[2]);

      for (int i = 0; i < variableChangedListeners.size(); i++)
      {
         VariableChangedListener listener = variableChangedListeners.get(i);
         listener.variableChanged(variable);
      }

   }

   @Override
   public void exitActionPerformed()
   {
      close();
   }


   private ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   public void attachVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.add(listener);
   }



}
