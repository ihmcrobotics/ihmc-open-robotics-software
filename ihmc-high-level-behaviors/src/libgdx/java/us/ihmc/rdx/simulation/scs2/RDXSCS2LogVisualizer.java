package us.ihmc.rdx.simulation.scs2;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.tools.RDXROS2StatsPanel;

/*
 * RDX UI to visualize SCS 2 logs with perception data from the robot runs.
 */
public class RDXSCS2LogVisualizer
{
   private final String logPath = "/home/duncan/Downloads/20250903_TennisBallMovingCrop/robotData.log";

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXPerceptionVisualizersPanel perceptionVisualizerPanel;
   private RDXSCS2LogSession scs2Session;

   public RDXSCS2LogVisualizer()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(3.0, 1.0, 2.5);

            baseUI.getImGuiPanelManager().addPanel(new RDXROS2StatsPanel());

            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();

            scs2Session = new RDXSCS2LogSession(baseUI);
            scs2Session.startSession(logPath, perceptionVisualizerPanel);

            perceptionVisualizerPanel.create(baseUI);
         }

         @Override
         public void render()
         {
            scs2Session.update();
            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            scs2Session.destroy(baseUI);
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSCS2LogVisualizer();
   }
}
