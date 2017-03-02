package us.ihmc.simulationconstructionset.simulationDispatcher.client.gui;

import java.awt.Color;
import java.awt.Graphics;

import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationToDispatch;


public class WaitingSimsPanel extends SimulationToDispatchPanel
{
   private static final long serialVersionUID = -1158350824241653239L;

   public WaitingSimsPanel(SimulationDispatcher dispatcher)
   {
      super(dispatcher);
   }

   @Override
   public void simulationsChanged()
   {
      SimulationToDispatch[] waitingSimulations = dispatcher.getSimulationsToDispatch();
      setSimulationsToDisplay(waitingSimulations);
   }

   @Override
   public void paintComponent(Graphics graphics)
   {
      super.paintComponent(graphics);

      graphics.setColor(Color.black);
      graphics.drawString("Description", 2, 10);

      int i = 0;
      for (SimulationToDispatch simulationToDispatch : simulations)
      {
         String description;

         if (simulationToDispatch != null)
            description = simulationToDispatch.getDescription();
         else
            description = "simulation";

         if (i == selectedSimulationIndex)
         {
            graphics.setColor(Color.magenta);
         }
         else
            graphics.setColor(Color.black);

         graphics.drawString(description, 2, i * FONT_SIZE + 22);

         i++;
      }

   }



}
