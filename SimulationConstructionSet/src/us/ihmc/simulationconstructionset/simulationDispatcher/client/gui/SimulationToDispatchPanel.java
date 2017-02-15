package us.ihmc.simulationconstructionset.simulationDispatcher.client.gui;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationToDispatch;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationsChangedListener;


public abstract class SimulationToDispatchPanel extends JPanel implements KeyListener, MouseListener, SimulationsChangedListener, Runnable
{
   private static final long serialVersionUID = 8160557287907988730L;
   protected SimulationDispatcher dispatcher;
   protected ConcurrentLinkedQueue<SimulationToDispatch> simulations = new ConcurrentLinkedQueue<SimulationToDispatch>();

   protected int selectedSimulationIndex = -1;
   protected SimulationToDispatch selectedSimulation;

   protected final static int FONT_SIZE = 12;

   public SimulationToDispatchPanel(SimulationDispatcher dispatcher)
   {
      this.dispatcher = dispatcher;

      setPanelSize(1);

      // hostsChanged();
      // this.setBorder(new SoftBevelBorder(BevelBorder.LOWERED));

      this.addMouseListener(this);
      this.addKeyListener(this);

      Thread anim = new Thread(this);
      anim.start();
   }

   @Override
   public void run()
   {
      while (true)
      {
         this.repaint();

         try
         {
            Thread.sleep(3000);
         }
         catch (InterruptedException e)
         {
         }
      }
   }

   protected void setSimulationsToDisplay(SimulationToDispatch[] simulationsToDisplay)
   {
      simulations.clear();

      for (int i = 0; i < simulationsToDisplay.length; i++)
      {
         simulations.add(simulationsToDisplay[i]);
      }

      // this.repaint();
      determineSelectedIndexFromSelectedSimulation();
      setPanelSize(simulationsToDisplay.length);
   }


   private void setPanelSize(int numberOfHosts)
   {
      int totalWidth = 12 * FONT_SIZE;
      Dimension dimension = new Dimension(totalWidth, FONT_SIZE * numberOfHosts + 22);

      this.setPreferredSize(dimension);
      this.setMaximumSize(dimension);
      this.updateUI();
   }


   @Override
   public void mousePressed(MouseEvent evt)
   {
   }

   @Override
   public void mouseReleased(MouseEvent evt)
   {
   }

   @Override
   public void mouseEntered(MouseEvent evt)
   {
   }

   @Override
   public void mouseExited(MouseEvent evt)
   {
   }


   @Override
   public void mouseClicked(MouseEvent evt)
   {
      int y = evt.getY();
      if (y < 0)
         return;
      selectedSimulationIndex = (y - 12) / FONT_SIZE;

      determineSelectedSimulationFromSelectedIndex();

      // System.out.print("Selected Host: ");
      // if (selectedHost != null) System.out.println(selectedHost.getHostName());

      this.requestFocus();
      this.repaint();

      // See if double click:

      /*
       * if ((evt.getClickCount() == 2) && (selectedVariable != null))
       * {
       * if (doubleClickListener != null) doubleClickListener.doubleClicked(selectedVariable);
       * }
       */
   }

   protected void determineSelectedSimulationFromSelectedIndex()
   {
      if (selectedSimulationIndex >= 0)
      {
         int index = 0;
         for (SimulationToDispatch simulationToDispatch : simulations)
         {
            if (index == selectedSimulationIndex)
            {
               selectedSimulation = simulationToDispatch;

               return;
            }

            index++;
         }
      }

      selectedSimulationIndex = -1;
      selectedSimulation = null;
   }

   protected void determineSelectedIndexFromSelectedSimulation()
   {
      if (selectedSimulation != null)
      {
         int index = 0;
         for (SimulationToDispatch simulationToDispatch : simulations)
         {
            if (simulationToDispatch == selectedSimulation)
            {
               selectedSimulationIndex = index;

               return;
            }

            index++;
         }
      }

      selectedSimulationIndex = -1;
      selectedSimulation = null;
   }


   public SimulationToDispatch getSelectedSimulation()
   {
      return this.selectedSimulation;
   }

   @Override
   public void keyTyped(KeyEvent evt)
   {
   }

   @Override
   public void keyReleased(KeyEvent evt)
   {
   }

   @Override
   public void keyPressed(KeyEvent keyEvent)
   {
      int code = keyEvent.getKeyCode();

      if (code == KeyEvent.VK_UP)
      {
         if (selectedSimulationIndex > 0)
            selectedSimulationIndex--;
      }

      else if (code == KeyEvent.VK_DOWN)
      {
         if (selectedSimulationIndex < simulations.size() - 1)
            selectedSimulationIndex++;
      }

      determineSelectedSimulationFromSelectedIndex();

      this.repaint();
   }

   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
   }

}
