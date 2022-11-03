package us.ihmc.rdx.simulation;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2BulletSimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.examples.simulations.bullet.FallingBoxBulletSimulation;
import us.ihmc.scs2.simulation.SimulationSession;

public class RDXSCS2BulletSimulationDemos
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private RDXSCS2BulletSimulationSession scs2SimulationSession;

   public RDXSCS2BulletSimulationDemos()
   {

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            SimulationSession fallingBoxSession = FallingBoxBulletSimulation.createSession();
            scs2SimulationSession = new RDXSCS2BulletSimulationSession(fallingBoxSession);
            scs2SimulationSession.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(scs2SimulationSession.getControlPanel());
         }

         @Override
         public void render()
         {
            scs2SimulationSession.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSCS2BulletSimulationDemos();
   }
}
