package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client.gui;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.net.URL;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ImageIcon;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationConstructionSetTools.simulationDispatcher.client.DispatchHost;
import us.ihmc.simulationConstructionSetTools.simulationDispatcher.client.SimulationToDispatch;
import us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces.RemoteSimulationRunnerInterface;


public class ShowSelectedAction extends AbstractAction implements Runnable
{
   private static final long serialVersionUID = 863990326301319460L;
   private URL iconURL = ShowSelectedAction.class.getClassLoader().getResource("icons/ShowSelected24.gif");
   private ImageIcon icon = new ImageIcon(iconURL);

   private String pwd;

   private DispatchHostPanel dispatchHostPanel;
   private SimulationDispatcherGUI GUI;

   public ShowSelectedAction(DispatchHostPanel dispatchHostPanel, SimulationDispatcherGUI GUI, String pwd)
   {
      super("Show Selected");

      this.pwd = pwd;

      // this.sim = sim;
      this.dispatchHostPanel = dispatchHostPanel;
      this.GUI = GUI;

      this.putValue(Action.SMALL_ICON, icon);
      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");

      Thread anim = new Thread(this);
      anim.start();
   }

   @Override
   public void actionPerformed(ActionEvent evt)
   {
      if (isPlaying)
      {
         isPlaying = false;
         System.out.println("Is Not Playing");

         return;
      }

      isPlaying = true;
      System.out.println("Is Playing");
   }

   private DispatchHost selectedHost = null;

   private void playCycle()
   {
      // System.out.println("Play Cycle");
      if (GUI == null)
         return;

      DispatchHost newHost = dispatchHostPanel.getSelectedHost();
      if (selectedHost != newHost)
         selectedHost = newHost;

      if (selectedHost == null)
      {
         // System.out.println("No host selected!");
         return;
      }

      // System.out.println("Selected Host: " + selectedHost.getHostName());

      SimulationToDispatch dispatchSim = selectedHost.getDispatchedSim();
      if (dispatchSim == null)
      {
         // System.out.println("Null Dispatch Sim");
         return;
      }

      Simulation sim = dispatchSim.getSimulation();
      if (sim == null)
      {
         // System.out.println("Simulation is null!");
         return;
      }

      RemoteSimulationRunnerInterface remoteSim = selectedHost.getRemoteSim();
      if (remoteSim == null)
      {
         // System.out.println("Remote Sim is null"); System.out.flush();
         return;
      }

      if (GUI != null)
         GUI.setSimulation(dispatchSim); 

      try
      {
         double[] data;

         // System.out.println("Getting Remote State"); System.out.flush();

         if (dispatchSim.isSimulationFinished())
            data = dispatchSim.getFinalState();
         else
            data = (double[]) remoteSim.getSimulationState(this.pwd);

         if (sim != dispatchSim.getSimulation())
            return;

         String[] outputStateVariableNames = dispatchSim.getOutputStateVariableNames();
         for (int i = 0; i < data.length; i++)
         {
            YoVariable var = sim.getVariable(outputStateVariableNames[i]);
            if (var != null)
               var.setValueFromDouble(data[i]);

            // System.out.println(data[i]);
         }

         GUI.updateRobot();
      }
      catch (Exception exc)
      {
         System.out.println("Remote Exception: " + exc.getMessage());
         exc.printStackTrace();
      }
   }


   private boolean isPlaying = false;

   @Override
   public void run()
   {
      while (true)
      {
         if (isPlaying)
            playCycle();

         try
         {
            Thread.sleep(300);
         }
         catch (InterruptedException e)
         {
         }
      }


   }
}
