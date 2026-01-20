package us.ihmc.rdx.ui.plotting;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2PlotPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2SimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.examples.simulations.bullet.StackOfBoxesExperimentalBulletSimulation;
import us.ihmc.scs2.session.SessionMode;

public class RDXSCS2StyleChartDemo
{
   private final RDXBaseUI baseUI;
   private RDXSCS2SimulationSession session;

   public RDXSCS2StyleChartDemo()
   {
      baseUI = new RDXBaseUI();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.values());

            session = new RDXSCS2SimulationSession(baseUI);
            session.startSession(StackOfBoxesExperimentalBulletSimulation.createSession());
            session.changeBufferDuration(1.0);
            session.getSession().setSessionMode(SessionMode.RUNNING);

            RDXSCS2PlotPanel plotPanel = session.getPlottingManager().addPlotPanel("Poses");
            plotPanel.setRows(7);
            plotPanel.setColumns(5);
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 0, "root.Block%d.q_Block%d_x".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 1, "root.Block%d.q_Block%d_y".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 2, "root.Block%d.q_Block%d_z".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 3, "root.Block%d.q_Block%d_qx".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 4, "root.Block%d.q_Block%d_qy".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 5, "root.Block%d.q_Block%d_qz".formatted(i, i));
            for (int i = 0; i < 5; i++)
               plotPanel.plot(i, 6, "root.Block%d.q_Block%d_qs".formatted(i, i));

            int block = 1;
            plotPanel.plot(0, 0, "root.Block%d.q_Block%d_x".formatted(block, block));
            plotPanel.plot(0, 0, "root.Block%d.q_Block%d_x".formatted(2, 2));
            plotPanel.plot(0, 1, "root.Block%d.q_Block%d_y".formatted(block, block));
            plotPanel.plot(0, 2, "root.Block%d.q_Block%d_z".formatted(block, block));
            plotPanel.plot(0, 3, "root.Block%d.q_Block%d_qx".formatted(block, block));
            plotPanel.plot(0, 4, "root.Block%d.q_Block%d_qy".formatted(block, block));
            plotPanel.plot(0, 5, "root.Block%d.q_Block%d_qz".formatted(block, block));
            plotPanel.plot(0, 6, "root.Block%d.q_Block%d_qs".formatted(block, block));
         }

         @Override
         public void render()
         {
            session.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            session.destroy(baseUI);
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSCS2StyleChartDemo();
   }
}
