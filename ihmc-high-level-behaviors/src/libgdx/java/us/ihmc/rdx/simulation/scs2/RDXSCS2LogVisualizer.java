package us.ihmc.rdx.simulation.scs2;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;

import java.nio.file.Path;
import java.nio.file.Paths;

/*
 * RDX UI to visualize SCS 2 logs with perception data.
 *
 * Pass -Dlog=~/LogDir to open that log on launch.
 */
public class RDXSCS2LogVisualizer
{
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
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(3.0, 1.0, 3.5);

            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();

            scs2Session = new RDXSCS2LogSession(baseUI, perceptionVisualizerPanel);

            String logDirectory = System.getProperty("log");
            if (logDirectory != null)
            {
               Path path = Paths.get(logDirectory.replaceAll("~", System.getProperty("user.home")), "robotData.log");
               if (path.toFile().exists())
                  scs2Session.startSession(path.toString());
            }

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
