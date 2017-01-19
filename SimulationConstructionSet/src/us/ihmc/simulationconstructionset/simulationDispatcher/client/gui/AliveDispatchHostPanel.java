package us.ihmc.simulationconstructionset.simulationDispatcher.client.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchHost;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchHostList;

public class AliveDispatchHostPanel extends DispatchHostPanel
{
   private static final long serialVersionUID = -3781923043775451428L;
   private static final int NAME_WIDTH = 12;
   private static final int DESCRIPTION_WIDTH = 12;
   private static final int NUMBER_WIDTH = 5;

   private static final int DESCRIPTION_START = NAME_WIDTH;
   private static final int ELAPSED_TIME_START = DESCRIPTION_START + DESCRIPTION_WIDTH;
   private static final int NUMBER_SIMS_START = ELAPSED_TIME_START + NUMBER_WIDTH;

   private static final int APPROX_TIME_START = NUMBER_SIMS_START + NUMBER_WIDTH;
   private static final int LAST_TIME_START = APPROX_TIME_START + NUMBER_WIDTH;

   private static final int TOTAL_WIDTH = (LAST_TIME_START + NUMBER_WIDTH) * FONT_SIZE;

   protected Dimension sz = new Dimension(TOTAL_WIDTH, FONT_SIZE);


   public AliveDispatchHostPanel(DispatchHostList dispatchHostList)
   {
      super(dispatchHostList);
      setPanelSize(1);
   }


   @Override
   public void hostsChanged()
   {
      dispatchHosts.clear();

      DispatchHost[] aliveHosts = dispatchHostList.getAllAliveHosts();
      for (int i = 0; i < aliveHosts.length; i++)
      {
         dispatchHosts.add(aliveHosts[i]);
      }

      // this.repaint();

      if (selectedHost == null)
         selected = -1;
      else if (dispatchHosts.contains(selectedHost))
      {
         selected = dispatchHosts.indexOf(selectedHost);
      }
      else
      {
         selectedHost = null;
         selected = -1;
      }

      setPanelSize(aliveHosts.length);
   }


   protected void setPanelSize(int nHosts)
   {
      sz.setSize(TOTAL_WIDTH, FONT_SIZE * nHosts + 22);
      this.setPreferredSize(sz);
      this.updateUI();
   }

   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);

      g.setColor(Color.black);

      g.drawString("Host Name", 2, 10);
      g.drawString("Description", 12 * (DESCRIPTION_START), 10);
      g.drawString("Elapsed", 12 * (ELAPSED_TIME_START), 10);
      g.drawString("Number", 12 * (NUMBER_SIMS_START), 10);
      g.drawString("Estimate", 12 * (APPROX_TIME_START), 10);
      g.drawString("Last", 12 * (LAST_TIME_START), 10);

      for (int i = 0; i < dispatchHosts.size(); i++)
      {
         DispatchHost dispatchHost = (DispatchHost) dispatchHosts.get(i);

         String hostName = dispatchHost.getHostName();
         String hostProcess = dispatchHost.getHostProcess();

         String description = null;
         if (dispatchHost.getDispatchedSim() != null)
            description = dispatchHost.getDispatchedSim().getDescription();
         String elapsedTime = String.valueOf(((int) dispatchHost.getTimeSinceLastSimulationStarted()));

         String numberSims = String.valueOf(dispatchHost.getNumberOfSimulationsRun());
         String approxTime = String.valueOf(dispatchHost.getApporxTimePerSimulation());
         String lastTime = String.valueOf(dispatchHost.getlastSimulationTime());

         if (i == selected)
            g.setColor(Color.magenta);
         else if (!dispatchHost.isAlive())
            g.setColor(Color.red);
         else if (dispatchHost.isRunningSim())
            g.setColor(Color.green);
         else
            g.setColor(Color.blue);

         g.drawString(hostName + ": " + hostProcess, 2, i * FONT_SIZE + 22);
         if (description != null)
            g.drawString(description, 12 * DESCRIPTION_START, i * FONT_SIZE + 22);
         g.drawString(elapsedTime, 12 * ELAPSED_TIME_START, i * FONT_SIZE + 22);
         g.drawString(numberSims, 12 * NUMBER_SIMS_START, i * FONT_SIZE + 22);
         g.drawString(approxTime, 12 * APPROX_TIME_START, i * FONT_SIZE + 22);
         g.drawString(lastTime, 12 * LAST_TIME_START, i * FONT_SIZE + 22);

      }
   }

}
