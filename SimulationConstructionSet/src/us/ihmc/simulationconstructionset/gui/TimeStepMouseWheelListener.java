package us.ihmc.simulationconstructionset.gui;

import java.awt.event.InputEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

/**
 * @author jcarff
 * Mouse Wheel Listener for time step incrementing
 */
public class TimeStepMouseWheelListener implements MouseWheelListener
{
   private StandardSimulationGUI standardSimulationGUI;


   public TimeStepMouseWheelListener(StandardSimulationGUI standardSimulationGUI)
   {
      this.standardSimulationGUI = standardSimulationGUI;
   }

   /*
    * on wheel up time step 1 forward on wheel down time step 1 back.
    * While holding control time steps are incremented by 20.
    */
   @Override
   public void mouseWheelMoved(MouseWheelEvent event)
   {
      if (event.getWheelRotation() < 0)
      {
         if (event.getModifiersEx() == InputEvent.CTRL_DOWN_MASK)
            standardSimulationGUI.stepForward(20);
         else
            standardSimulationGUI.stepForward();
      }
      else
      {
         if (event.getModifiersEx() == InputEvent.CTRL_DOWN_MASK)
            standardSimulationGUI.stepBackward(20);
         else
            standardSimulationGUI.stepBackward();
      }
   }
   
   public void closeAndDispose()
   {
      standardSimulationGUI = null;
   }
}
