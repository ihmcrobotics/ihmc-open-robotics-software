package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

/**
 * Created by Peter on 8/29/2017.
 */
public class JSliderParameterControl extends JPanel implements CloseableAndDisposable, MouseListener
{
   private static int MIN_VALUE = 0;
   private static final int MAX_VALUE = 100;
   private YoVariable yoVariable;

   private static final long serialVersionUID = 8570638563928914747L;

   private boolean changeLock = false;
   private boolean updatedRemotly = false;
   private JTextField value;
   private JSlider jSlider;

   private ParameterSliderWindow parameterSliderWindow;

   private String name;

   private SelectedVariableHolder selectedVariableHolder;

   public JSliderParameterControl(SelectedVariableHolder selectedVariableHolder, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      super(new BorderLayout());
      //TODO: how to handle the different types of Parameters and the possible min/max values?

      jSlider = new JSlider(SwingConstants.VERTICAL, MIN_VALUE, MAX_VALUE, 50);

      yoVariable = null;
      setTitle();

      jSlider.setPaintLabels(false);

      this.name = name;

      this.selectedVariableHolder = selectedVariableHolder;


      JTextField jTextField = new JTextField(String.valueOf(1));

      this.add(jSlider, BorderLayout.CENTER);

      this.setName("garbage");
      //slider.set value = new JTextField(slider.midiControl.var.getNumericValueAsAString());

      this.add(jTextField, BorderLayout.SOUTH);


      closeableAndDisposableRegistry.registerCloseableAndDisposable(this);

      jSlider.addMouseListener(this);


   }

   public String getName()
   {
      if (yoVariable != null)
      {
         return yoVariable.getName();
      }
      else
      {
         return "Not Set";
      }
   }

   @Override
   public void closeAndDispose()
   {

   }

   private void setTitle()
   {
      this.setBorder(new TitledBorder(this.getName()));
      this.repaint();
   }

   synchronized public void lock()
   {
      if (!changeLock)
      {
         Thread timer = new Thread(new Runnable()
         {
            @Override
            public void run()
            {
               changeLock = true;

               try
               {
                  Thread.sleep(1000);
               }
               catch (InterruptedException e)
               {
               }

               changeLock = false;
            }
         });
         timer.start();
      }
   }

   @Override
   public void mouseClicked(MouseEvent e)
   {
   }

   @Override
   public void mousePressed(MouseEvent e)
   {
      if (e.getButton() == MouseEvent.BUTTON1)
      {
         //System.out.println(", BUTTON1");
      }
      else if (e.getButton() == MouseEvent.BUTTON2)
      {
         YoVariable yoVariable = selectedVariableHolder.getSelectedVariable();

         if (yoVariable != null && yoVariable.isParameter())
         {
            this.yoVariable = yoVariable;
            setTitle();
         }


      }
      else if (e.getButton() == MouseEvent.BUTTON3)
      {
         //System.out.println(", BUTTON3");
      }
   }

   @Override
   public void mouseReleased(MouseEvent e)
   {
      //System.out.print("mouseReleased in slider " + name);
   }

   @Override
   public void mouseEntered(MouseEvent e)
   {
   }

   @Override
   public void mouseExited(MouseEvent e)
   {
   }
}
