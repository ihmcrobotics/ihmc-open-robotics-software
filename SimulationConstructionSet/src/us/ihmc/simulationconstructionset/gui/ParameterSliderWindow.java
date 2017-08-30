package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Created by Peter on 8/29/2017.
 */


public class ParameterSliderWindow implements CloseableAndDisposable
{
   private static final int MAX_CHANNELS_PER_ROW = 13;

   private ArrayList<JSliderParameterControl> sliderParameterControls = new ArrayList<JSliderParameterControl>();

   private JPanel mainPanel = new JPanel();

   private JFrame frame = new JFrame();

   private int numRow = 0;
   private int numCol = 0;
   private static final int sliderWidth = 80;
   private static final int sliderHeight = 300;

   private VariableChangedListener listener;
   private final Object slidersLock = new Object();
   private ReentrantLock controlsLock = new ReentrantLock();

   private CloseableAndDisposableRegistry closeableAndDisposableRegistry;

   private final SelectedVariableHolder selectedVariableHolder;

   public ParameterSliderWindow(SelectedVariableHolder selectedVariableHolder, final CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      this.closeableAndDisposableRegistry = closeableAndDisposableRegistry;
      this.selectedVariableHolder = selectedVariableHolder;

      System.out.println("Parameter Slider");

      frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
      frame.getContentPane().setLayout(new BorderLayout());
      frame.getContentPane().add(mainPanel, BorderLayout.CENTER);
      frame.setTitle("Parameter Slider");
      frame.setVisible(true);

      setUpGui();

//      sliderBoard.addListener(new SliderBoardControlAddedListener()
//      {
//         @Override
//         public void controlAdded(MidiControl ctrl)
//         {
//            addSlider(ctrl);
//         }
//
//         @Override
//         public void controlRemoved(MidiControl ctrl)
//         {
//            removeSlider(ctrl, closeableAndDisposableRegistry);
//         }
//      });
//
//      listener = new VariableChangedListener()
//      {
//         @Override
//         public void variableChanged(YoVariable<?> v)
//         {
//            synchronized (VirtualSliderBoardGui.this)
//            {
//               synchronized (slidersLock)
//               {
//
//                  for (JSliderControl tmpControl : sliders)
//                  {
//                     if (tmpControl.getControl().var == v)
//                     {
//                        updateSlider(tmpControl);
//                     }
//                  }
//               }
//            }
//         }
//      };

      // Thread valueCheckerThread = new Thread(new ValueChecker());
      // valueCheckerThread.start();

      //closeableAndDisposableRegistry.registerCloseableAndDisposable(this);
   }

//   private synchronized void addSlider(MidiControl ctrl)
//   {
//      virtualMidiControls.add(ctrl);
//      ctrl.var.addVariableChangedListener(listener);
//      setUpGui(closeableAndDisposableRegistry);
//   }
//
//   private void removeSlider(MidiControl midiControl, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
//   {
//      controlsLock.lock();
//      virtualMidiControls.remove(midiControl);
//
//      if (midiControl.var.getVariableChangedListeners().contains(listener))
//      {
//         midiControl.var.removeVariableChangedListener(listener);
//      }
//
//      setUpGui(closeableAndDisposableRegistry);
//   }


   public void repaint()
   {
      mainPanel.repaint();
   }

   private void setUpGui()
   {
      synchronized (slidersLock)
      {
         sliderParameterControls.clear();
         mainPanel.removeAll();

         int numberOfSliders = 3;
         for (int i=0; i< numberOfSliders; i++)
         {
            JSliderParameterControl jSliderParameterControl;
            jSliderParameterControl = new JSliderParameterControl(selectedVariableHolder, closeableAndDisposableRegistry);


//            if (current.sliderType == MidiControl.SliderType.BOOLEAN)
//            {
//               slider = new JSliderControl(SwingConstants.VERTICAL, 0, 1, SliderBoardUtils.valueRatioConvertToIntWithExponents(current, 0), current,
//                                           closeableAndDisposableRegistry);
//            } else if (current.sliderType == MidiControl.SliderType.ENUM)
//            {
//               slider = new JSliderControl(SwingConstants.VERTICAL, 0, ((YoEnum<?>) current.var).getEnumValues().length - 1,
//                                           SliderBoardUtils.valueRatioConvertToIntWithExponents(current,
//                                                                                                ((YoEnum<?>) current.var).getEnumValues().length - 1), current, closeableAndDisposableRegistry);
//            } else
//               slider = new JSliderControl(SwingConstants.VERTICAL, 0, sliderBoardMax,
//                                           SliderBoardUtils.valueRatioConvertToIntWithExponents(current, sliderBoardMax), current, closeableAndDisposableRegistry);


            //slider.addChangeListener(new SliderChangeListener(slider, this));


//            if (slider.midiControl.name != null)
//            {
//               sliderPanel.setBorder(new TitledBorder(slider.midiControl.name));
//            }
//            else
//            {
//               sliderPanel.setBorder(new TitledBorder(slider.midiControl.var.getName()));
//            }





            //slider.value.addActionListener(new textFieldListener(slider));
            //sliderPanel.add(slider.value, BorderLayout.SOUTH);
            sliderParameterControls.add(jSliderParameterControl);

            numRow = Math.round(sliderParameterControls.size() / MAX_CHANNELS_PER_ROW);

            numCol = sliderParameterControls.size();
            if (numRow > 1)
               numCol = MAX_CHANNELS_PER_ROW;

            if (sliderParameterControls.size() % MAX_CHANNELS_PER_ROW != 0)
               numRow++;

            mainPanel.setLayout(new GridLayout(numRow, numCol));

            mainPanel.add(jSliderParameterControl, sliderParameterControls.size() - 1);
         }

         resizePanel();
      }
   }

//   private synchronized void updateSlider(JSliderControl slider)
//   {
//      if (!slider.changeLock)
//      {
//         slider.updatedRemotly = true;
//
//         // slider.ctrl.currentVal = sliderVal;
//         slider.setValue(SliderBoardUtils.valueRatioConvertToIntWithExponents(slider.midiControl, slider.sliderMax));
//
//         String formattedValue = new DecimalFormat("#.##").format(slider.midiControl.var.getValueAsDouble());
//
//         slider.value.setText(formattedValue);
//      }
//   }

//   private synchronized void sliderSlid(JSlider slider, double sliderVal)
//   {
//      if (!slider.updatedRemotly)
//      {
//         // channel is between 0 and 15. value is between 0 and 127.
//         if (slider.midiControl.currentVal == sliderVal)
//            return;
//         slider.midiControl.currentVal = sliderVal;
//         slider.lock();
//         slider.midiControl.var.setValueFromDouble(sliderVal);
//
//         for (VariableChangedListener listener : sliderBoard.getVariableChangedListeners())
//         {
//            listener.variableChanged(slider.midiControl.var);
//         }
//
//         // System.out.println(slider.ctrl.var.getValueAsDouble()+" " +SliderBoardUtils.valueRatioConvertToIntWithExponents(slider.ctrl, slider.sliderMax));
//
//         slider.setValue(SliderBoardUtils.valueRatioConvertToIntWithExponents(slider.midiControl, slider.sliderMax));
//
//         String formattedValue = new DecimalFormat("#.##").format(slider.midiControl.var.getValueAsDouble());
//
//         slider.value.setText(formattedValue);
//      }
//      else
//         slider.updatedRemotly = false;
//   }

//   private class SliderChangeListener implements ChangeListener
//   {
//      JSliderControl slider;
//      VirtualSliderBoardGui board;
//
//      public SliderChangeListener(JSliderControl slider, VirtualSliderBoardGui board)
//      {
//         this.board = board;
//         this.slider = slider;
//      }
//
//      @Override
//      public void stateChanged(ChangeEvent e)
//      {
//         // System.out.println();
//         board.sliderSlid(slider, SliderBoardUtils.valueRatioConvertToDoubleWithExponents(slider.midiControl, slider.getValue(), slider.sliderMax));
//      }
//   }


   private void resizePanel()
   {
      synchronized (slidersLock)
      {
         frame.setSize(sliderWidth * (sliderParameterControls.size() / numRow), sliderHeight * numRow);
      }
   }

   public void setTitle(String name)
   {
      frame.setTitle(name);
   }

   public void setFrameLocation(int x, int y)
   {
      frame.setLocation(x, y);
   }

   @Override
   public void closeAndDispose()
   {
      sliderParameterControls.clear();
      sliderParameterControls = null;

      listener = null;

      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            mainPanel.setVisible(false);
            mainPanel = null;

            frame.setVisible(false);
            frame.dispose();
            frame = null;
         }
      });
   }

   //   public class TextFieldListener implements ActionListener
//   {
//      JSlider slider;
//
//      public TextFieldListener(JSlider slider)
//      {
//         this.slider = slider;
//      }
//
//      @Override
//      public void actionPerformed(ActionEvent e)
//      {
//         try
//         {
//            Double val = new Double(slider.value.getText());
//            sliderSlid(slider, val);
//         }
//         catch (Exception ex)
//         {
//            System.err.println("An Invalid Value Was Entered Onto The Virtual SliderBoard");
//         }
//      }
//   }
}
