package us.ihmc.simulationconstructionsettools.util.inputdevices;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class VirtualSliderBoardGui implements CloseableAndDisposable
{
   private static final int MAX_CHANNELS_PER_ROW = 13;

   private static final int sliderBoardMax = 127;

   private ArrayList<MidiControl> virtualMidiControls = new ArrayList<MidiControl>();

   private ArrayList<JPanel> sliderPanels = new ArrayList<JPanel>();

   private JPanel mainPanel = new JPanel();

   private JFrame frame = new JFrame();
   private ArrayList<JSliderControl> sliders = new ArrayList<JSliderControl>();

   private int numRow = 0;
   private int numCol = 0;
   private static final int sliderWidth = 80;
   private static final int sliderHeight = 300;

   private MidiSliderBoard sliderBoard;
   private VariableChangedListener listener;
   private final Object slidersLock = new Object();
   private ReentrantLock controlsLock = new ReentrantLock();

   private CloseableAndDisposableRegistry closeableAndDisposableRegistry;

   public VirtualSliderBoardGui(MidiSliderBoard sliderBoard, final CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {

      System.out.println("CREATING VIRTUAL SLIDERBOARD");
      this.closeableAndDisposableRegistry = closeableAndDisposableRegistry;

      this.sliderBoard = sliderBoard;
      frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
      frame.getContentPane().setLayout(new BorderLayout());
      frame.getContentPane().add(mainPanel, BorderLayout.CENTER);
      frame.setTitle("Virtual Slider Board");
      frame.setVisible(true);

      sliderBoard.addListener(new SliderBoardControlAddedListener()
      {
         @Override
         public void controlAdded(MidiControl ctrl)
         {
            addSlider(ctrl);
         }

         @Override
         public void controlRemoved(MidiControl ctrl)
         {
            removeSlider(ctrl, closeableAndDisposableRegistry);
         }
      });

      listener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            synchronized (VirtualSliderBoardGui.this)
            {
               synchronized (slidersLock)
               {

                  for (JSliderControl tmpControl : sliders)
                  {
                     if (tmpControl.getControl().var == v)
                     {
                        updateSlider(tmpControl);
                     }
                  }
               }
            }
         }
      };

      // Thread valueCheckerThread = new Thread(new ValueChecker());
      // valueCheckerThread.start();

      closeableAndDisposableRegistry.registerCloseableAndDisposable(this);
   }

   @Override
   public void closeAndDispose()
   {
      sliderPanels.clear();
      sliderPanels = null;

      virtualMidiControls.clear();
      virtualMidiControls = null;

      sliders.clear();
      sliders = null;

      sliderBoard = null;
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

   private synchronized void addSlider(MidiControl ctrl)
   {
      virtualMidiControls.add(ctrl);
      ctrl.var.addVariableChangedListener(listener);
      setUpGui(closeableAndDisposableRegistry);
   }

   private void removeSlider(MidiControl midiControl, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      controlsLock.lock();
      virtualMidiControls.remove(midiControl);

      if (midiControl.var.getVariableChangedListeners().contains(listener))
      {
         midiControl.var.removeVariableChangedListener(listener);
      }

      setUpGui(closeableAndDisposableRegistry);
   }

   private void setUpGui(CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      synchronized (slidersLock)
      {
         sliderPanels.clear();
         sliders.clear();
         mainPanel.removeAll();

         for (MidiControl current : virtualMidiControls)
         {
            JSliderControl slider;
            if (current.sliderType == MidiControl.SliderType.BOOLEAN)
            {
               slider = new JSliderControl(SwingConstants.VERTICAL, 0, 1, SliderBoardUtils.valueRatioConvertToIntWithExponents(current, 0), current,
                       closeableAndDisposableRegistry);
            } else if (current.sliderType == MidiControl.SliderType.ENUM)
            {
               slider = new JSliderControl(SwingConstants.VERTICAL, 0, ((EnumYoVariable<?>) current.var).getEnumValues().length - 1,
                       SliderBoardUtils.valueRatioConvertToIntWithExponents(current,
                               ((EnumYoVariable<?>) current.var).getEnumValues().length - 1), current, closeableAndDisposableRegistry);
            } else
               slider = new JSliderControl(SwingConstants.VERTICAL, 0, sliderBoardMax,
                       SliderBoardUtils.valueRatioConvertToIntWithExponents(current, sliderBoardMax), current, closeableAndDisposableRegistry);

            slider.setPaintLabels(false);
            slider.addChangeListener(new SliderChangeListener(slider, this));

            JPanel sliderPanel = new JPanel(new BorderLayout());
            if (slider.midiControl.name != null)
            {
               sliderPanel.setBorder(new TitledBorder(slider.midiControl.name));
            }
            else
            {
               sliderPanel.setBorder(new TitledBorder(slider.midiControl.var.getName()));
            }
            sliderPanel.add(slider, BorderLayout.CENTER);
            slider.value = new JTextField(slider.midiControl.var.getNumericValueAsAString());
            sliders.add(slider);
            slider.value.addActionListener(new textFieldListener(slider));
            sliderPanel.add(slider.value, BorderLayout.SOUTH);
            sliderPanels.add(sliderPanel);

            numRow = Math.round(sliderPanels.size() / MAX_CHANNELS_PER_ROW);
            numCol = sliderPanels.size();
            if (numRow > 1)
               numCol = MAX_CHANNELS_PER_ROW;

            if (sliderPanels.size() % MAX_CHANNELS_PER_ROW != 0)
               numRow++;

            mainPanel.setLayout(new GridLayout(numRow, numCol));

            mainPanel.add(sliderPanel, sliderPanels.size() - 1);
         }

         resizePanel();
      }
   }

   private synchronized void updateSlider(JSliderControl slider)
   {
      if (!slider.changeLock)
      {
         slider.updatedRemotly = true;

         // slider.ctrl.currentVal = sliderVal;
         slider.setValue(SliderBoardUtils.valueRatioConvertToIntWithExponents(slider.midiControl, slider.sliderMax));

         String formattedValue = new DecimalFormat("#.##").format(slider.midiControl.var.getValueAsDouble());

         slider.value.setText(formattedValue);
      }
   }

   private synchronized void sliderSlid(JSliderControl slider, double sliderVal)
   {
      if (!slider.updatedRemotly)
      {
         // channel is between 0 and 15. value is between 0 and 127.
         if (slider.midiControl.currentVal == sliderVal)
            return;
         slider.midiControl.currentVal = sliderVal;
         slider.lock();
         slider.midiControl.var.setValueFromDouble(sliderVal);

         for (VariableChangedListener listener : sliderBoard.getVariableChangedListeners())
         {
            listener.variableChanged(slider.midiControl.var);
         }

         // System.out.println(slider.ctrl.var.getValueAsDouble()+" " +SliderBoardUtils.valueRatioConvertToIntWithExponents(slider.ctrl, slider.sliderMax));

         slider.setValue(SliderBoardUtils.valueRatioConvertToIntWithExponents(slider.midiControl, slider.sliderMax));

         String formattedValue = new DecimalFormat("#.##").format(slider.midiControl.var.getValueAsDouble());

         slider.value.setText(formattedValue);
      }
      else
         slider.updatedRemotly = false;
   }

   private class SliderChangeListener implements ChangeListener
   {
      JSliderControl slider;
      VirtualSliderBoardGui board;

      public SliderChangeListener(JSliderControl slider, VirtualSliderBoardGui board)
      {
         this.board = board;
         this.slider = slider;
      }

      @Override
      public void stateChanged(ChangeEvent e)
      {
         // System.out.println();
         board.sliderSlid(slider, SliderBoardUtils.valueRatioConvertToDoubleWithExponents(slider.midiControl, slider.getValue(), slider.sliderMax));
      }
   }


   private void resizePanel()
   {
      synchronized (slidersLock)
      {
         frame.setSize(sliderWidth * (sliders.size() / numRow), sliderHeight * numRow);
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

   public class textFieldListener implements ActionListener
   {
      JSliderControl slider;

      public textFieldListener(JSliderControl slider)
      {
         this.slider = slider;
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         try
         {
            Double val = new Double(slider.value.getText());
            sliderSlid(slider, val);
         }
         catch (Exception ex)
         {
            System.err.println("An Invalid Value Was Entered Onto The Virtual SliderBoard");
         }
      }
   }
}
