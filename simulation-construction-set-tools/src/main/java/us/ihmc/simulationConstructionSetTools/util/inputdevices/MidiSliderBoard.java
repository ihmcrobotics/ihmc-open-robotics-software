package us.ihmc.simulationConstructionSetTools.util.inputdevices;

import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Enumeration;
import java.util.Hashtable;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;

import us.ihmc.commons.PrintTools;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.sliderboardVisualizer.MidiSliderBoardConfigurationVisualizer;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class MidiSliderBoard implements ExitActionListener, CloseableAndDisposable
{
   // There are problems with using both virtual and physical sliderboards at the same time when connected to the biped.
   private boolean alwaysShowVirtualSliderBoard = false;

   private enum Devices
   {
      VIRTUAL, BCF2000, XTOUCH_COMPACT, GENERIC
   }

   public static final int CHECK_TIME = 10;

   // public int sliderOffset = 0;
   public int sliderBoardMax = 127;

   protected Hashtable<Integer, MidiControl> controlsHashTable = new Hashtable<Integer, MidiControl>(40);
   protected ArrayList<SliderListener> internalListeners = new ArrayList<SliderListener>();
   private ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   private ArrayList<SliderBoardControlAddedListener> controlAddedListeners = new ArrayList<SliderBoardControlAddedListener>();

   private Devices preferedDevice = Devices.VIRTUAL;
   private int preferdDeviceNumber = -1;

   private static final boolean DEBUG = false;

   private MidiDevice inDevice = null;
   private Receiver midiOut = null;
   private SliderBoardTransmitterInterface transmitter = null;
   private VirtualSliderBoardGui virtualSliderBoard;
   private MidiSliderBoardConfigurationVisualizer visualizer;
   private VariableChangedListener listener;
   private YoVariableHolder holder;
   
   private MidiChannelMapper channelMapper = new GenericChannelMapper();

   private CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   public MidiSliderBoard(SimulationConstructionSet scs)
   {
      this(scs, true, false);
   }

   public MidiSliderBoard(SimulationConstructionSet scs, boolean showVirtualSliderBoard)
   {
      this(scs, showVirtualSliderBoard, false);
   }

   public MidiSliderBoard(SimulationConstructionSet scs, boolean showVirtualSliderBoard, boolean showSliderBoardVisualizer)
   {
      this((YoVariableHolder) scs);
      boolean showSCSWindows = scs != null && scs.getSimulationConstructionSetParameters().getShowWindows();
      if ((showSCSWindows && preferedDevice.equals(Devices.VIRTUAL)) || (showVirtualSliderBoard || alwaysShowVirtualSliderBoard))
      {
         
         if ((scs == null) || (scs.getStandardSimulationGUI() != null))
         {
            PrintTools.info(this, "Setting Up Virtual Slider Board");
            virtualSliderBoard = new VirtualSliderBoardGui(this, closeableAndDisposableRegistry);
         }
      }
      else if (!preferedDevice.equals(Devices.VIRTUAL) && showSliderBoardVisualizer)
      {
         visualizer = new MidiSliderBoardConfigurationVisualizer(controlsHashTable);
      }

      if (scs != null)
      {
         printIfDebug("Attaching exit action listener " + getClass().getSimpleName() + " to SCS");
         scs.attachExitActionListener(this);
      }
   }

   public MidiSliderBoard(YoVariableHolder scs)
   {
      try
      {
         MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();

         // ArrayList<Integer> inDeviceNumbers = new ArrayList<Integer>();

         // findPotentialDeviceNumbers(infos, inDeviceNumbers, printInfo);

         init(infos);

         // if no devices are found

         if (scs != null)
         {
            this.holder = scs;
         }

         // if a motorized slider board is found
         if (preferedDevice.equals(Devices.BCF2000))
         {
            PrintTools.info("Setting Up Motorized Slider Board");
            channelMapper = new BCF2000ChannelMapper();

            // sliderOffset = 80;
            sliderBoardMax = 127;

            try
            {
               inDevice.getTransmitter().setReceiver(new BCF2000Receiver(controlsHashTable, internalListeners, variableChangedListeners, sliderBoardMax, this));

               transmitter = new BCF2000Transmitter(midiOut, sliderBoardMax);
            }
            catch (Exception e)
            {
               if (DEBUG)
                  System.err.println("Exception when trying to get MIDI transmitter 1: " + e);
            }

            final Object self = this;
            listener = new VariableChangedListener()
            {
               @Override
               public void notifyOfVariableChange(YoVariable<?> v)
               {
                  synchronized (self)
                  {
                     for (MidiControl tmpControl : controlsHashTable.values())
                     {
                        if (tmpControl.var.equals(v))
                        {
                           double value = 0.0;
                           value = (tmpControl).var.getValueAsDouble();
                           yoVariableChanged((tmpControl).mapping, value);

                           if (visualizer != null)
                              visualizer.updateValue(tmpControl, value);
                        }
                     }
                  }
               }
            };

            // Thread valueCheckerThread = new Thread(new ValueChecker());
            // valueCheckerThread.start();
         }

         // if a regular slider board is found
         else if (preferedDevice.equals(Devices.GENERIC))
         {
            // sliderOffset = -1;

            System.out.println("Setting Up Physical Slider Board");
            channelMapper = new BCF2000ChannelMapper(); // Code was the same when making this
            sliderBoardMax = 127;

            try
            {
               inDevice.getTransmitter().setReceiver(new UC33Receiver(controlsHashTable, internalListeners, variableChangedListeners, sliderBoardMax));
            }
            catch (Exception e)
            {
               if (DEBUG)
                  System.err.println("Exception when trying to get MIDI transmitter 1: " + e);
            }
         }
         else if (preferedDevice.equals(Devices.XTOUCH_COMPACT))
         {
            PrintTools.info("Setting Up XTouch Slider Board");
            channelMapper = new XTouchChannelMapper();

            // sliderOffset = 80;
            sliderBoardMax = 127;

            try
            {
               inDevice.getTransmitter().setReceiver(new XTouchCompactReceiver(controlsHashTable, internalListeners, variableChangedListeners, sliderBoardMax, this));

               transmitter = new XTouchCompactTransmitter(midiOut, sliderBoardMax);
            }
            catch (Exception e)
            {
               if (DEBUG)
                  System.err.println("Exception when trying to get MIDI transmitter 1: " + e);
            }

            final Object self = this;
            listener = new VariableChangedListener()
            {
               @Override
               public void notifyOfVariableChange(YoVariable<?> v)
               {
                  synchronized (self)
                  {
                     for (MidiControl tmpControl : controlsHashTable.values())
                     {
                        if (tmpControl.var.equals(v))
                        {
                           double value = 0.0;
                           value = (tmpControl).var.getValueAsDouble();
                           yoVariableChanged((tmpControl).mapping, value);

                           if (visualizer != null)
                              visualizer.updateValue(tmpControl, value);
                        }
                     }
                  }
               }
            };

         }

         addListener(new SliderListener()
         {
            @Override
            public void valueChanged(MidiControl midiControl)
            {
               if (DEBUG)
                  System.out.println("EVL::valueChanged [" + midiControl.mapping + "] value to : " + midiControl.var.getValueAsDouble() + " YoVal:"
                        + midiControl.var.getValueAsDouble());
               if (visualizer != null)
                  visualizer.updateValue(midiControl);
            }
         });
      }
      catch (Exception e)
      {
         e.printStackTrace();

      }
   }

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
      virtualSliderBoard = null;

      holder = null;

      controlsHashTable.clear();
      controlsHashTable = null;

      internalListeners.clear();
      internalListeners = null;

      variableChangedListeners.clear();
      variableChangedListeners = null;

      controlAddedListeners.clear();
      controlAddedListeners = null;

      if (inDevice != null)
      {
         try
         {
            inDevice.close();
         }
         catch (Exception e)
         {
            System.err.println("Exception when trying to close inDevice in MidiSliderBoard.closeAndDispose()");
         }
      }

      if (midiOut != null)
      {
         try
         {
            midiOut.close();
         }
         catch (Exception e)
         {
            System.err.println("Exception when trying to close midiOut in MidiSliderBoard.closeAndDispose()");
         }
      }

      if (transmitter != null)
      {
         transmitter.closeAndDispose();
      }

      virtualSliderBoard = null;
      inDevice = null;
      midiOut = null;
   }

   public void setTitle(String title)
   {
      if (virtualSliderBoard != null)
         virtualSliderBoard.setTitle(title);
      if (visualizer != null)
         visualizer.setTitle(title);
   }

   public void createLabels()
   {
      if (visualizer != null)
         visualizer.createLabels();
   }

   public int init(MidiDevice.Info[] infos)
   {
      MidiDevice outDevice = null;
      if (DEBUG)
         System.out.println("EvolutionUC33E::init found " + infos.length + " MIDI device infos.");

      int rc = 0;

      String name = null;
      String description = null;
      MidiDevice current = null;
      for (int i = 0; i < infos.length; i++)
      {
         if (DEBUG)
         {
            System.out.println("  Device[" + i + "] " + infos[i].getName());
            System.out.println("          Vendor: " + infos[i].getVendor());
            System.out.println("         Version: " + infos[i].getVersion());
            System.out.println("     Description: " + infos[i].getDescription());
         }

         name = infos[i].getName();
         description = infos[i].getDescription();

         if ((name.indexOf("UC-33") >= 0) || (infos[i].getDescription().indexOf("UC-33") >= 0))
         {
            if (preferdDeviceNumber == -1)
            {
               preferdDeviceNumber = i;

               if (DEBUG)
                  System.out.println("Found Generic SliderBoard");
               preferedDevice = Devices.GENERIC;

               try
               {
                  inDevice = MidiSystem.getMidiDevice(infos[i]);
                  System.out.println("PHYSICAL " + inDevice.getDeviceInfo().getName() + " - " + inDevice.getDeviceInfo().getDescription());
               }
               catch (MidiUnavailableException e)
               {
                  // TODO Auto-generated catch block
                  e.printStackTrace();
               }

               try
               {
                  if (DEBUG)
                  {
                     System.out.println("\nGot device with info" + inDevice.getDeviceInfo());
                     System.out.println("Max Receivers:" + inDevice.getMaxReceivers());
                     System.out.println("Max Transmitters:" + inDevice.getMaxTransmitters());
                  }

                  if (inDevice.isOpen())
                  {
                     if (DEBUG)
                        System.out.println("\nDevice started open. Closing Device\n");
                     inDevice.close();

                     if (inDevice.isOpen())
                     {
                        System.err.println("\nMIDI device started open. Attempted to close it, but could not. You may need to reboot to close the device.");
                     }
                  }

                  else
                  {
                     if (DEBUG)
                        System.out.println("Device started closed");
                  }

                  if (DEBUG)
                     System.out.println("\nOpening Device\n");

                  try
                  {
                     inDevice.open();
                  }
                  catch (Exception e)
                  {
                     if (DEBUG)
                        System.out.println("Exception while trying to open device: " + e);
                  }

               }

               catch (Exception e)
               {
                  System.out.println("Exception: " + e);
               }
            }
         }
         else if (name.contains("BCF2000") || description.contains("BCF2000"))
         {
            preferdDeviceNumber = i;
            preferedDevice = Devices.BCF2000;
            if (DEBUG)
               System.out.println("Found motorizedSliderBoard");

            try
            {
               current = MidiSystem.getMidiDevice(infos[i]);
            }
            catch (MidiUnavailableException e)
            {
               if (DEBUG)
               {
                  System.out.println("   - Unable to get a handle to this Midi Device.");
                  e.printStackTrace();
               }

               continue;
            }

            if ((outDevice == null) && (current.getMaxReceivers() != 0))
            {
               outDevice = current;

               if (!outDevice.isOpen())
               {
                  if (DEBUG)
                     System.out.println("   - Opening Output Device");

                  try
                  {
                     outDevice.open();
                  }
                  catch (MidiUnavailableException e)
                  {
                     outDevice = null;

                     if (DEBUG)
                     {
                        System.out.println("   - Unable to open device.");
                        e.printStackTrace();
                     }

                     continue;
                  }
               }

               if (DEBUG)
                  System.out.println("   - Device is Now open trying to obtain the receiver.");

               try
               {
                  midiOut = outDevice.getReceiver();
               }
               catch (MidiUnavailableException e)
               {
                  outDevice = null;
                  midiOut = null;

                  if (DEBUG)
                  {
                     System.out.println("   - Error getting the device's receiver.");
                     e.printStackTrace();
                  }

                  continue;
               }

               if (DEBUG)
                  System.out.println("   - Obtained a handle to the devices receiver.");
               rc += 2;
            }

            if ((inDevice == null) && (current.getMaxTransmitters() != 0))
            {
               inDevice = current;

               if (DEBUG)
               {
                  System.out.println("\nGot device with info" + inDevice.getDeviceInfo());
                  System.out.println("Max Receivers:" + inDevice.getMaxReceivers());
                  System.out.println("Max Transmitters:" + inDevice.getMaxTransmitters());
               }

               if (DEBUG)
                  System.out.println("   - Opening Input Device.");

               try
               {
                  inDevice.open();
               }
               catch (MidiUnavailableException e1)
               {
                  inDevice = null;

                  if (DEBUG)
                  {
                     System.out.println("   - Exception while trying to open device.");
                     e1.printStackTrace();
                  }

                  continue;
               }

               if (DEBUG)
                  System.out.println("   - Device is Now open trying to obtain the transmitter.");

               rc += 1;
            }
         }
         else if (description.contains("X-TOUCH COMPACT"))
         {
            preferdDeviceNumber = i;
            preferedDevice = Devices.XTOUCH_COMPACT;
            if (DEBUG)
               System.out.println("Found X-Touch motorizedSliderBoard");

            try
            {
               current = MidiSystem.getMidiDevice(infos[i]);
            }
            catch (MidiUnavailableException e)
            {
               if (DEBUG)
               {
                  System.out.println("   - Unable to get a handle to this Midi Device.");
                  e.printStackTrace();
               }

               continue;
            }

            if ((outDevice == null) && (current.getMaxReceivers() != 0))
            {
               outDevice = current;

               if (!outDevice.isOpen())
               {
                  if (DEBUG)
                     System.out.println("   - Opening Output Device");

                  try
                  {
                     outDevice.open();
                  }
                  catch (MidiUnavailableException e)
                  {
                     outDevice = null;

                     if (DEBUG)
                     {
                        System.out.println("   - Unable to open device.");
                        e.printStackTrace();
                     }

                     continue;
                  }
               }

               if (DEBUG)
                  System.out.println("   - Device is Now open trying to obtain the receiver.");

               try
               {
                  midiOut = outDevice.getReceiver();
               }
               catch (MidiUnavailableException e)
               {
                  outDevice = null;
                  midiOut = null;

                  if (DEBUG)
                  {
                     System.out.println("   - Error getting the device's receiver.");
                     e.printStackTrace();
                  }

                  continue;
               }

               if (DEBUG)
                  System.out.println("   - Obtained a handle to the devices receiver.");
               rc += 2;
            }

            if ((inDevice == null) && (current.getMaxTransmitters() != 0))
            {
               inDevice = current;

               if (DEBUG)
               {
                  System.out.println("\nGot device with info" + inDevice.getDeviceInfo());
                  System.out.println("Max Receivers:" + inDevice.getMaxReceivers());
                  System.out.println("Max Transmitters:" + inDevice.getMaxTransmitters());
               }

               if (DEBUG)
                  System.out.println("   - Opening Input Device.");

               try
               {
                  inDevice.open();
               }
               catch (MidiUnavailableException e1)
               {
                  inDevice = null;

                  if (DEBUG)
                  {
                     System.out.println("   - Exception while trying to open device.");
                     e1.printStackTrace();
                  }

                  continue;
               }

               if (DEBUG)
                  System.out.println("   - Device is Now open trying to obtain the transmitter.");

               rc += 1;
            }
         }         
      }

      return rc;
   }

   public void setButton(int channel, String name, YoVariableHolder holder)
   {
      setButton(channel, holder.getVariable(name));
   }

   public void setButtonEnum(int channel, String name, YoVariableHolder holder, Enum<?> enumValue)
   {
      setButtonEnum(channel, (YoEnum<?>) holder.getVariable(name), enumValue);
   }

   public void setButton(int channel, String varName, String buttonName, YoVariableHolder holder)
   {
      setButton(channel, holder.getVariable(varName), buttonName);
   }

   public void setButton(int channel, YoVariable<?> var)
   {
 
      setControl(channelMapper.getButtonChannel(channel), var, 0, 1, 1, MidiControl.SliderType.BOOLEAN, MidiControl.ControlType.BUTTON);
   }

   public void setButtonEnum(int channel, YoEnum<?> yoEnum, Enum<?> enumValue)
   {
       setControl(channelMapper.getButtonChannel(channel), yoEnum, 0.0, enumValue.ordinal(), 1.0, MidiControl.SliderType.ENUM, MidiControl.ControlType.BUTTON);
   }

   public void setButton(int channel, YoVariable<?> var, String name)
   {
      setControl(channelMapper.getButtonChannel(channel), var, name, 0, 1, 1, MidiControl.SliderType.BOOLEAN, MidiControl.ControlType.BUTTON);
   }

   public void setKnobButton(int channel, String name, YoVariableHolder holder)
   {
      setKnobButton(channel, holder.getVariable(name));
   }

   public void setKnobButton(int channel, String varName, String buttonName, YoVariableHolder holder)
   {
      setKnobButton(channel, holder.getVariable(varName), buttonName);
   }

   public void setKnobButton(int channel, YoVariable<?> var)
   {
      setControl(channelMapper.getKnobButtonChannel(channel), var, 0, 1, 1, MidiControl.SliderType.BOOLEAN, MidiControl.ControlType.BUTTON);
   }

   public void setKnobButton(int channel, YoVariable<?> var, String name)
   {
      setControl(channelMapper.getKnobButtonChannel(channel), var, name, 0, 1, 1, MidiControl.SliderType.BOOLEAN, MidiControl.ControlType.BUTTON);
   }

   public void setSlider(int channel, String name, YoVariableHolder holder, double min, double max)
   {
      setSlider(channel, name, holder, min, max, 1.0);
   }

   public void setSlider(int channel, String varName, String sliderName, YoVariableHolder holder, double min, double max)
   {
      setSlider(channel, varName, sliderName, holder, min, max, 1.0);
   }

   public void setSlider(int channel, String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      if (!holder.hasUniqueVariable(name))
      {
         PrintTools.warn("Trying to add yovariable to slider, but it does not exist, or more than 1 exists: " + name);
      }

      YoVariable<?> variable = holder.getVariable(name);
      if (variable == null)
      {
         PrintTools.error("Ahhhhhhhhh, yoVariable name " + name + " was not found in the registry. It's not getting added to the sliderboard");
         return;
      }
      setSlider(channel, variable, min, max, exponent);
   }

   public void setSlider(int channel, String varName, String sliderName, YoVariableHolder holder, double min, double max, double exponent)
   {
      if (!holder.hasUniqueVariable(varName))
      {
         PrintTools.warn("Trying to add yovariable to slider, but it does not exist, or more than 1 exists: " + varName);
      }

      YoVariable<?> variable = holder.getVariable(varName);
      if (variable == null)
      {
         PrintTools.error("Ahhhhhhhhh, yoVariable name " + varName + " was not found in the registry. It's not getting added to the sliderboard");
         return;
      }

      setSlider(channel, holder.getVariable(varName), sliderName, min, max, exponent);
   }

   public void setSlider(int channel, String name, double min, double max, double exponent, double hires)
   {
      setSlider(channel, name, holder, min, max, exponent, hires);
   }

   public void setSlider(int channel, String varName, String sliderName, double min, double max, double exponent, double hires)
   {
      setSlider(channel, varName, sliderName, holder, min, max, exponent, hires);
   }

   public void setSlider(int channel, String name, YoVariableHolder holder, double min, double max, double exponent, double hires)
   {
      if (!holder.hasUniqueVariable(name))
      {
         PrintTools.warn("Trying to add yovariable to slider, but it does not exist, or more than 1 exists: " + name);
      }

      setSlider(channel, holder.getVariable(name), min, max, exponent, hires);
   }

   public void setSlider(int channel, String varName, String sliderName, YoVariableHolder holder, double min, double max, double exponent, double hires)
   {
      if (!holder.hasUniqueVariable(varName))
      {
         PrintTools.warn("Trying to add yovariable to slider, but it does not exist, or more than 1 exists: " + varName);
      }

      setSlider(channel, holder.getVariable(varName), sliderName, min, max, exponent, hires);
   }

   public void setSlider(int channel, YoVariable<?> var, double min, double max)
   {
      setSlider(channel, var, min, max, 1.0);
   }

   public void setSlider(int channel, YoVariable<?> var, String name, double min, double max)
   {
      setSlider(channel, var, name, min, max, 1.0);
   }

   public void setKnob(int channel, String name, YoVariableHolder holder, double min, double max)
   {
      setKnob(channel, name, holder, min, max, 1.0);
   }

   public void setKnob(int channel, String varName, String knobName, YoVariableHolder holder, double min, double max)
   {
      setKnob(channel, varName, knobName, holder, min, max, 1.0);
   }

   public void setKnob(int channel, String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      if (!holder.hasUniqueVariable(name))
      {
         PrintTools.warn("Trying to add yovariable to knob, but it does not exist, or more than 1 exists: " + name);
      }

      setKnob(channel, holder.getVariable(name), min, max, exponent);
   }

   public void setKnob(int channel, String varName, String knobName, YoVariableHolder holder, double min, double max, double exponent)
   {
      if (!holder.hasUniqueVariable(varName))
      {
         PrintTools.warn("Trying to add yovariable to knob, but it does not exist, or more than 1 exists: " + varName);
      }

      setKnob(channel, holder.getVariable(varName), knobName, min, max, exponent);
   }

   public void setKnob(int channel, String name, YoVariableHolder holder, double min, double max, double exponent, double hires)
   {
      if (!holder.hasUniqueVariable(name))
      {
         PrintTools.warn("Trying to add yovariable to knob, but it does not exist, or more than 1 exists: " + name);
      }

      setKnob(channel, holder.getVariable(name), min, max, exponent, hires);
   }

   public void setKnob(int channel, String varName, String knobName, YoVariableHolder holder, double min, double max, double exponent, double hires)
   {
      if (!holder.hasUniqueVariable(varName))
      {
         PrintTools.warn("Trying to add yovariable to knob, but it does not exist, or more than 1 exists: " + varName);
      }

      setKnob(channel, holder.getVariable(varName), knobName, min, max, exponent, hires);
   }

   public void setKnob(int channel, YoVariable<?> var, double min, double max)
   {
      setKnob(channel, var, min, max, 1.0);
   }

   public void setKnob(int channel, YoVariable<?> var, String name, double min, double max)
   {
      setKnob(channel, var, name, min, max, 1.0);
   }

   public void setKnobEnum(int channel, YoEnum<?> var)
   {
      setControl(channelMapper.getKnobChannel(channel), var, 0.0, var.getEnumValues().length - 1, 1.0, MidiControl.SliderType.ENUM, MidiControl.ControlType.KNOB);
   }

   public void setSliderEnum(int channel, String name, YoVariableHolder holder)
   {
      setSliderEnum(channel, (YoEnum<?>) holder.getVariable(name));
   }

   public void setSliderEnum(int channel, String varName, String sliderName, YoVariableHolder holder)
   {
      setSliderEnum(channel, (YoEnum<?>) holder.getVariable(varName), sliderName);
   }

   public void setSliderEnum(int channel, YoEnum<?> var)
   {
      setControl(channelMapper.getSliderChannel(channel), var, 0.0, var.getEnumValues().length - 1, 1.0, MidiControl.SliderType.ENUM, MidiControl.ControlType.SLIDER);
   }

   public void setSliderEnum(int channel, YoEnum<?> var, String name)
   {
      setControl(channelMapper.getSliderChannel(channel), var, name, 0.0, var.getEnumValues().length - 1, 1.0, MidiControl.SliderType.ENUM, MidiControl.ControlType.SLIDER);
   }

   public void setSliderBoolean(int channel, String name, YoVariableHolder holder)
   {
      setSliderBoolean(channel, holder.getVariable(name));
   }

   public void setSliderBoolean(int channel, String varName, String sliderName, YoVariableHolder holder)
   {
      setSliderBoolean(channel, holder.getVariable(varName), sliderName);
   }

   public void setSliderBoolean(int channel, YoVariable<?> var)
   {
      setControl(channelMapper.getSliderChannel(channel), var, 0.0, 1.0, 1.0, MidiControl.SliderType.BOOLEAN, MidiControl.ControlType.SLIDER);
   }

   public void setSliderBoolean(int channel, YoVariable<?> var, String name)
   {
      setControl(channelMapper.getSliderChannel(channel), var, name, 0.0, 1.0, 1.0, MidiControl.SliderType.BOOLEAN, MidiControl.ControlType.SLIDER);
   }

   public void setSlider(int channel, YoVariable<?> var, double min, double max, double exponent)
   {
      setControl(channelMapper.getSliderChannel(channel), var, min, max, exponent, MidiControl.SliderType.NUMBER, MidiControl.ControlType.SLIDER);
   }

   public void setSlider(int channel, YoVariable<?> var, String name, double min, double max, double exponent)
   {
      setControl(channelMapper.getSliderChannel(channel), var, name, min, max, exponent, MidiControl.SliderType.NUMBER, MidiControl.ControlType.SLIDER);
   }

   public void setSlider(int channel, YoVariable<?> var, double min, double max, double exponent, double hires)
   {
      if (var == null)
      {
         PrintTools.error(this, "YoVariable was null. It's not getting added to the sliderboard");
         return;
      }
      setControl(channelMapper.getSliderChannel(channel), var, var.getName(), min, max, exponent, hires, MidiControl.SliderType.NUMBER, MidiControl.ControlType.SLIDER);
   }

   public void setSlider(int channel, YoVariable<?> var, String name, double min, double max, double exponent, double hires)
   {
      setControl(channelMapper.getSliderChannel(channel), var, name, min, max, exponent, hires, MidiControl.SliderType.NUMBER, MidiControl.ControlType.SLIDER);
   }

   public void setKnob(int channel, YoVariable<?> var, double min, double max, double exponent)
   {
      setControl(channelMapper.getKnobChannel(channel), var, min, max, exponent, MidiControl.SliderType.NUMBER, MidiControl.ControlType.KNOB);
   }

   public void setKnob(int channel, YoVariable<?> var, String name, double min, double max, double exponent)
   {
      setControl(channelMapper.getKnobChannel(channel), var, name, min, max, exponent, MidiControl.SliderType.NUMBER, MidiControl.ControlType.KNOB);
   }

   public void setKnob(int channel, YoVariable<?> var, double min, double max, double exponent, double hires)
   {
      if (var == null)
      {
         PrintTools.error(this, "YoVariable was null. It's not getting added to the sliderboard");
         return;
      }
      setControl(channelMapper.getKnobChannel(channel), var, var.getName(), min, max, exponent, hires, MidiControl.SliderType.NUMBER, MidiControl.ControlType.KNOB);
   }

   public void setKnob(int channel, YoVariable<?> var, String name, double min, double max, double exponent, double hires)
   {
      setControl(channelMapper.getKnobChannel(channel), var, name, min, max, exponent, hires, MidiControl.SliderType.NUMBER, MidiControl.ControlType.KNOB);
   }

   private void setControl(int channel, YoVariable<?> var, String name, double min, double max, double exponent, MidiControl.SliderType sliderType, MidiControl.ControlType controlType)
   {
      setControl(channel, var, name, min, max, exponent, (min + max) / 2.0, sliderType, controlType);
   }

   private void setControl(int channel, YoVariable<?> var, double min, double max, double exponent, MidiControl.SliderType sliderType, MidiControl.ControlType controlType)
   {

      if (var == null)
      {
         PrintTools.error("YoVariable was null. It's not getting added to the sliderboard");
         return;
      }
      setControl(channel, var, var.getName(), min, max, exponent, (min + max) / 2.0, sliderType, controlType);
   }

   private synchronized void setControl(int channel, YoVariable<?> var, String name, double min, double max, double exponent, double hires,
                                        MidiControl.SliderType sliderType, MidiControl.ControlType controlType)
   {
      if (var != null)
      {
         if (exponent <= 0.0)
         {
            System.err.println("Slider Board: Exponent must be positive. Setting it to 1.0");
            exponent = 1.0;
         }

         MidiControl midiControl = new MidiControl(channel, var, max, min, exponent, hires, name);
         midiControl.sliderType = sliderType;
         midiControl.controlType = controlType;

         if (listener != null)
         {
            var.addVariableChangedListener(listener);
         }

         setControl(midiControl);
         setToInitialPosition(midiControl);

         for (SliderBoardControlAddedListener listener : controlAddedListeners)
         {
            listener.controlAdded(midiControl);
         }
      }
      else
      {
         // PrintTools.error("Passed in null variable");
      }
   }

   public synchronized void addListOfControls(Collection<MidiControl> collection)
   {
      for (MidiControl control : collection)
      {
         setControl(control);
         setToInitialPosition(control);

         for (SliderBoardControlAddedListener listener : controlAddedListeners)
         {
            listener.controlAdded(control);
         }
      }

      if (visualizer != null)
      {
         visualizer.createLabels();
      }
   }

   public void setRange(int channel, double min, double max)
   {
      setRange(channel, min, max, 1.0);
   }

   public void setRange(int channel, double min, double max, double exponent)
   {
      if (exponent <= 0.0)
      {
         System.err.println("Peavey PC1600X: Exponent must be positive. Setting it to 1.0");
         exponent = 1.0;
      }

      MidiControl control = controlsHashTable.get(channel);
      control.min = min;
      control.max = max;
      control.exponent = exponent;
   }

   private synchronized void setControl(MidiControl midiControl)
   {
      controlsHashTable.put(midiControl.mapping, midiControl);
   }

   public synchronized void clearControls()
   {
      for (MidiControl toBeRemoved : controlsHashTable.values())
      {
         for (SliderBoardControlAddedListener listener : controlAddedListeners)
         {
            listener.controlRemoved(toBeRemoved);
         }

      }

      controlsHashTable.clear();
      if (visualizer != null)
         visualizer.clearLabels();
   }

   protected void moveControl(MidiControl midiControl)
   {
      if (transmitter != null)
      {
         transmitter.moveControl(midiControl);
      }
   }

   protected void moveControl(MidiControl midiControl, int sliderValue)
   {
      if (transmitter != null)
      {
         transmitter.moveControl(midiControl, sliderValue);
      }

   }

   public void setToInitialPosition(MidiControl midiControl)
   {
      // ctrl.var.setValueFromDouble(ctrl.reset);

      for (SliderListener listener : internalListeners)
      {
         listener.valueChanged(midiControl);
      }

      moveControl(midiControl);
   }

   public void reset()
   {
      MidiControl midiControl = null;
      Enumeration<MidiControl> midiControllers = controlsHashTable.elements();
      while (midiControllers.hasMoreElements())
      {
         midiControl = midiControllers.nextElement();
         midiControl.var.setValueFromDouble(midiControl.reset);

         for (SliderListener listener : internalListeners)
         {
            listener.valueChanged(midiControl);
         }

         moveControl(midiControl);
      }
   }

   public double getValue(int mapping)
   {
      MidiControl midiControl = controlsHashTable.get(mapping);
      if (midiControl != null)
         return midiControl.var.getValueAsDouble();

      return -1;
   }

   public void setValue(int mapping, double value) throws InvalidParameterException
   {
      MidiControl midiControl = controlsHashTable.get(mapping);
      if (midiControl == null)
         throw new InvalidParameterException("name does not map to a control");

      if (midiControl.currentVal == value)
         return;

      midiControl.currentVal = value;

      for (SliderListener listener : internalListeners)
      {
         listener.valueChanged(midiControl);
      }

      moveControl(midiControl);
   }

   public void yoVariableChanged(int mapping, double value) throws InvalidParameterException
   {
      MidiControl midiControl = controlsHashTable.get(mapping);
      if (midiControl == null)
         throw new InvalidParameterException("name does not map to a control");

      moveControl(midiControl);
   }

   public void addListener(SliderListener listener)
   {
      internalListeners.add(listener);
   }

   public void removeListener(SliderListener listener)
   {
      internalListeners.remove(listener);
   }

   public void addListener(SliderBoardControlAddedListener listener)
   {
      controlAddedListeners.add(listener);
   }

   public void removeListener(SliderBoardControlAddedListener listener)
   {
      controlAddedListeners.remove(listener);
   }

   public void attachVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.add(listener);
   }

   public ArrayList<VariableChangedListener> getVariableChangedListeners()
   {
      return new ArrayList<VariableChangedListener>(variableChangedListeners);
   }

   @Override
   public void exitActionPerformed()
   {
      printIfDebug("Exit Action was performed. Closing and disposing " + getClass().getSimpleName());

      this.closeAndDispose();
   }

   public interface SliderListener
   {
      public void valueChanged(MidiControl midiControl);
   }

   public void setVirtualSliderBoardFrameLocation(int x, int y)
   {
      virtualSliderBoard.setFrameLocation(x, y);
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }
}
