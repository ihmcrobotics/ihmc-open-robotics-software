package us.ihmc.simulationconstructionset.gui;

import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

import javax.swing.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

/**
 * Created by Peter on 8/29/2017.
 */
public class JSliderParameterControl extends JSlider implements CloseableAndDisposable, MouseListener
{
   private static int MIN_VALUE = 0;
   private static final int MAX_VALUE = 100;
   private Parameter parameter;

   private static final long serialVersionUID = 8570638563928914747L;

   private boolean changeLock = false;
   private boolean updatedRemotly = false;
   private JTextField value;

   private String name;

   public JSliderParameterControl(String name, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      //TODO: how to handle the different types of Parameters and the possible min/max values?
      super(SwingConstants.VERTICAL, MIN_VALUE, MAX_VALUE, 50);

      this.name = name;

      closeableAndDisposableRegistry.registerCloseableAndDisposable(this);
   }

   public String getName()
   {
      return name;
   }

   @Override
   public void closeAndDispose()
   {

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
      System.out.println("mouseClicked in slider " + name);
   }

   @Override
   public void mousePressed(MouseEvent e)
   {
      System.out.println("mousePressed in slider " + name);
   }

   @Override
   public void mouseReleased(MouseEvent e)
   {
      System.out.println("mouseReleased in slider " + name);
   }

   @Override
   public void mouseEntered(MouseEvent e)
   {
      System.out.println("mouseEntered in slider " + name);
   }

   @Override
   public void mouseExited(MouseEvent e)
   {
      System.out.println("mouseExited in slider " + name);
   }
}
