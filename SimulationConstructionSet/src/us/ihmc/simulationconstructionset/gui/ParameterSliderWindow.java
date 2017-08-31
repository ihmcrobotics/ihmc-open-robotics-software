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
   private static final int MAX_CHANNELS_PER_ROW = 8;

   private JPanel mainPanel = new JPanel();

   private JFrame frame = new JFrame();

   private int numRow = 0;
   private int numCol = 0;
   private static final int sliderWidth = 80;
   private static final int sliderHeight = 300;

   private final Object slidersLock = new Object();

   private CloseableAndDisposableRegistry closeableAndDisposableRegistry;

   private final SelectedVariableHolder selectedVariableHolder;
   private int numberOfSliders;

   public ParameterSliderWindow(SelectedVariableHolder selectedVariableHolder, final CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      this(MAX_CHANNELS_PER_ROW, selectedVariableHolder, closeableAndDisposableRegistry);
   }

   public ParameterSliderWindow(int numberOfSliders, SelectedVariableHolder selectedVariableHolder,
                                final CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      this.numberOfSliders = numberOfSliders;
      this.numCol = Math.min(numberOfSliders, MAX_CHANNELS_PER_ROW);
      this.numRow = (int) Math.ceil(numberOfSliders / (double) MAX_CHANNELS_PER_ROW);
      this.closeableAndDisposableRegistry = closeableAndDisposableRegistry;
      this.selectedVariableHolder = selectedVariableHolder;

      frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
      frame.getContentPane().setLayout(new BorderLayout());
      frame.getContentPane().add(mainPanel, BorderLayout.CENTER);
      frame.setTitle("Parameter Slider");

      mainPanel.removeAll();
      mainPanel.setLayout(new GridLayout(numRow, numCol));

      for (int i = 0; i < numberOfSliders; i++)
      {
         mainPanel.add(new JSliderParameterControl(selectedVariableHolder, closeableAndDisposableRegistry));
      }

      frame.setVisible(true);
      frame.pack();
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
}
