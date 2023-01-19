package us.ihmc.rdx.simulation;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2BulletSimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletFlyingBallSimulationTest;

public class RDXSCS2FlyingBallOrbitalEnergyTest
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private RDXSCS2BulletSimulationSession scs2SimulationSession;

   public RDXSCS2FlyingBallOrbitalEnergyTest()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            scs2SimulationSession = new RDXSCS2BulletSimulationSession();
            scs2SimulationSession.create(baseUI);
            scs2SimulationSession.startSession(new BulletFlyingBallSimulationTest().createSession());
            scs2SimulationSession.changeBufferDuration(20.0);
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
      new RDXSCS2FlyingBallOrbitalEnergyTest();
   }
}
