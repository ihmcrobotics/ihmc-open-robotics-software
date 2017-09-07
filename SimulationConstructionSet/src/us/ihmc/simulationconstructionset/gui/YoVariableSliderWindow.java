package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

import javax.swing.*;
import java.awt.*;

/**
 * Created by Peter on 8/29/2017.
 */

public class YoVariableSliderWindow implements CloseableAndDisposable
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

   public YoVariableSliderWindow(SelectedVariableHolder selectedVariableHolder, final CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      this(MAX_CHANNELS_PER_ROW, selectedVariableHolder, closeableAndDisposableRegistry);
   }

   public YoVariableSliderWindow(int numberOfSliders, SelectedVariableHolder selectedVariableHolder,
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
      frame.setTitle("YoVariable Slider");

      mainPanel.removeAll();
      mainPanel.setLayout(new GridLayout(numRow, numCol));

      for (int i = 0; i < numberOfSliders; i++)
      {
         mainPanel.add(new JSliderYoVariableControl(selectedVariableHolder, closeableAndDisposableRegistry));
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
