package us.ihmc.rdx.simulation;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2BulletSimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.examples.simulations.CrossFourBarLinkageDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

/**
 * Currently fails to launch with Name collision for new variable: wrist_btappliedforcex.
 * Parent name space = CrossFourBarLinkageRobot.CrossFourBarLinkageRobotBulletRobot
 *
 * This is probably because BulletRobot and BulletRobotLinkJoint need to handle four bars.
 */
public class RDXSCS2FourBarLinkageTest
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private RDXSCS2BulletSimulationSession scs2SimulationSession;

   public RDXSCS2FourBarLinkageTest()
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

//            SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);
            SimulationSession simulationSession = new SimulationSession();
            CrossFourBarLinkageDefinition robotDefinition = new CrossFourBarLinkageDefinition();
            simulationSession.addRobot(robotDefinition);

            scs2SimulationSession.startSession(simulationSession);
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
      new RDXSCS2FourBarLinkageTest();
   }
}
