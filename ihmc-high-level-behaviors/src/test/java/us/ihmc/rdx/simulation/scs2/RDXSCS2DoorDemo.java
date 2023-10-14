package us.ihmc.rdx.simulation.scs2;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.behaviors.simulation.FlatGroundDefinition;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorDefinition;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * An SCS 2 simulation of a door hinged on a frame with a lever handle.
 */
public class RDXSCS2DoorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXSCS2Session visualOnlySession;

   public RDXSCS2DoorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            visualOnlySession = new RDXSCS2Session();
            SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);

            DoorDefinition doorDefinition = new DoorDefinition();
            doorDefinition.getDoorPanelDefinition().setAddArUcoMarkers(true);
            doorDefinition.build();

            // Pull door
//            doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(1.0, -0.5, 0.01));

            // Push door
            doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(Math.PI, 0.0, 0.0), new Point3D(1.3, 0.5, 0.01));
            doorDefinition.getInitialHingeState().setEffort(15.0);
            Robot doorRobot = simulationSession.addRobot(doorDefinition);
            doorDefinition.applyPDController(doorRobot);

            simulationSession.addTerrainObject(new FlatGroundDefinition());

            visualOnlySession.create(baseUI);
            visualOnlySession.startSession(simulationSession);
            baseUI.getPrimaryScene().addRenderableProvider(visualOnlySession::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(visualOnlySession.getControlPanel());
         }

         @Override
         public void render()
         {
            visualOnlySession.update();

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
      new RDXSCS2DoorDemo();
   }
}
