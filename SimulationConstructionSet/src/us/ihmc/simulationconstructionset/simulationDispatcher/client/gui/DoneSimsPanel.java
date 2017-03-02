package us.ihmc.simulationconstructionset.simulationDispatcher.client.gui;

import java.awt.Color;
import java.awt.Graphics;

import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationToDispatch;


public class DoneSimsPanel extends SimulationToDispatchPanel
{
   private static final long serialVersionUID = 4247621299985751757L;


   public DoneSimsPanel(SimulationDispatcher dispatcher)
   {
      super(dispatcher);
   }

   @Override
   public void simulationsChanged()
   {
      SimulationToDispatch[] doneSimulations = dispatcher.getDoneSimulations();
      setSimulationsToDisplay(doneSimulations);
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
         String resultsString;

         if (simulationToDispatch != null)
         {
            description = simulationToDispatch.getDescription();
            resultsString = simulationToDispatch.getResultsString();
         }
         else
         {
            description = "simulation";
            resultsString = null;
         }

         if (resultsString != null)
         {
            description = description + " " + resultsString;
         }

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
