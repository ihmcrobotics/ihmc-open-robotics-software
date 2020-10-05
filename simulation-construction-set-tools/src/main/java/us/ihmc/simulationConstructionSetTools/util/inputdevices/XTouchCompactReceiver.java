package us.ihmc.simulationConstructionSetTools.util.inputdevices;

import java.util.ArrayList;
import java.util.Hashtable;

import javax.sound.midi.MidiMessage;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;

import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiControl.ControlType;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;

public class XTouchCompactReceiver implements Receiver
{
   public int SLIDERBOARDMAX = 127;

   protected Hashtable<Integer, MidiControl> controlsHashTable;
   protected ArrayList<MidiSliderBoard.SliderListener> internalListeners;
   private ArrayList<YoVariableChangedListener> variableChangedListeners;
   private boolean debug = false;
   private MidiSliderBoard board;

   public XTouchCompactReceiver(Hashtable<Integer, MidiControl> controlsHashTable, ArrayList<MidiSliderBoard.SliderListener> internalListeners,
         ArrayList<YoVariableChangedListener> variableChangedListeners, int SliderBoardMax, MidiSliderBoard board)
   {
      this.board = board;
      this.SLIDERBOARDMAX = SliderBoardMax;
      this.controlsHashTable = controlsHashTable;
      this.internalListeners = internalListeners;
      this.variableChangedListeners = variableChangedListeners;
   }

   @Override
   public void close()
   {
   }

   @Override
   public void send(MidiMessage message, long timeStamp)
   {
      if (message instanceof ShortMessage)
      {
         ShortMessage sm = (ShortMessage) message;
         if (debug)
         {
            System.out.println("*************************");
            System.out.println("Channel " + sm.getChannel());
            System.out.println("Command " + sm.getCommand());
            System.out.println("data1 " + sm.getData1());
            System.out.println("data2 " + sm.getData2());
            System.out.println("Status " + sm.getStatus());
         }

         int control = sm.getData1();
         if(sm.getCommand() == 128 || sm.getCommand() == 144)
         {
            control += 127;
         }
         
         MidiControl ctrl = controlsHashTable.get(control);
         if (ctrl != null)
         {
            
            
            if(ctrl.controlType == ControlType.SLIDER || ctrl.controlType == ControlType.KNOB) // Sliders & Knobs
            {
               double newVal = 0.0;
   
               newVal = SliderBoardUtils.valueRatioConvertToDoubleWithExponents(ctrl, sm.getData2(), SLIDERBOARDMAX);
   
               ctrl.currentVal = newVal;
               ctrl.var.setValueFromDouble(newVal);
   
               // snap it in place
               if (ctrl.sliderType == MidiControl.SliderType.BOOLEAN)
               {
                  //System.out.println("newVal: "+newVal);
                  if (newVal <= 0.5)
                  {
                     //System.out.println("moving to 0");
                     board.moveControl(ctrl, 0);
                  } else
                  {
                     //System.out.println("movint to "+SLIDERBOARDMAX);
                     board.moveControl(ctrl, SLIDERBOARDMAX);
                  }
               }
   
               else if (ctrl.sliderType == MidiControl.SliderType.ENUM)
               {
                  board.moveControl(ctrl, SliderBoardUtils.valueRatioConvertToIntWithExponents(ctrl, SLIDERBOARDMAX));
               }
   
               // ctrl.var.notifyObservers(new Event(EventType.CHANGED));
               
               notifyListeners(ctrl);
   
            }
            else if(ctrl.controlType == ControlType.BUTTON)
            {
               if(sm.getCommand() == 128)
               {
                  boolean currentVal = ctrl.var.getValueAsDouble() > 0.5;
                  if(currentVal)
                  {
                     ctrl.var.setValueFromDouble(0.0);
                  }
                  else
                  {
                     ctrl.var.setValueFromDouble(1.0);
                  }
                  notifyListeners(ctrl);

               }
               else
               {
                  // Ignore button release
               }
            }
            else
            {
               System.err.println("Unknown control type " + ctrl.controlType);
            }
         }
      }
   }

   private void notifyListeners(MidiControl ctrl)
   {
      for (MidiSliderBoard.SliderListener listener : internalListeners)
      {
         listener.valueChanged(ctrl);
      }

      if (ctrl.notify)
      {
         for (int i = 0; i < variableChangedListeners.size(); i++)
         {
            YoVariableChangedListener listener = variableChangedListeners.get(i);
            listener.changed(ctrl.var);
         }
      }

      if (debug)
         System.out.println("Changed [" + ctrl.mapping + "] value to : " + ctrl.var.getValueAsDouble());

   }

}
