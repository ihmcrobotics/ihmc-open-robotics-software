package us.ihmc.rdx.simulation;

import us.ihmc.exampleSimulations.fourBarLinkage.CrossFourBarLinkageRobotDefinition;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2SimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;

/**
 * Currently fails to launch with Name collision for new variable: wrist_btappliedforcex.
 * Parent name space = CrossFourBarLinkageRobot.CrossFourBarLinkageRobotBulletRobot
 *
 * This is probably because BulletRobot and BulletRobotLinkJoint need to handle four bars.
 */
public class RDXSCS2FourBarLinkageTest
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXSCS2SimulationSession scs2SimulationSession;

   public RDXSCS2FourBarLinkageTest()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            scs2SimulationSession = new RDXSCS2SimulationSession(baseUI);

            SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);
            CrossFourBarLinkageRobotDefinition robotDefinition = new CrossFourBarLinkageRobotDefinition();
            simulationSession.addRobot(robotDefinition);

            scs2SimulationSession.startSession(simulationSession);
            scs2SimulationSession.changeBufferDuration(20.0);
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
