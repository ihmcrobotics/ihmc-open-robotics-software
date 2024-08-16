package us.ihmc.rdx.simulation.scs2;

import us.ihmc.behaviors.simulation.FlatGroundDefinition;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.behaviors.simulation.door.DoorDefinition;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.robots.RDXDoorWidgets;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * An SCS 2 simulation of a door hinged on a frame with a lever handle.
 */
public class RDXSCS2DoorDemo extends Lwjgl3ApplicationAdapter
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXSCS2RestartableSimulationSession rdxSimulationSession;
   private final RDXDoorWidgets doorWidgets = new RDXDoorWidgets();

   public RDXSCS2DoorDemo()
   {
      baseUI.launchRDXApplication(this);
   }

   @Override
   public void create()
   {
      baseUI.create();
      baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);
      baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.7, 0.0, 0.4));
      baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, -4.0, 4.0);

      rdxSimulationSession = new RDXSCS2RestartableSimulationSession(baseUI);
      rdxSimulationSession.setSessionBuilder(this::buildSession);
      rdxSimulationSession.getAdditionalImGuiWidgets().add(() ->
      {
         doorWidgets.renderImGuiWidgets();
      });
      rdxSimulationSession.getOnSessionStartedRunnables().add(() ->
      {
         doorWidgets.zeroTorques();
         rdxSimulationSession.getBulletPhysicsDebugger().setUpdateDebugDrawings(true);
         rdxSimulationSession.getSession().runTick();
         rdxSimulationSession.setPauseAtEndOfBuffer(false);
         rdxSimulationSession.getSession().setSessionMode(SessionMode.RUNNING);
      });

      rdxSimulationSession.buildSimulation();
   }

   @Override
   public void render()
   {
      rdxSimulationSession.update();

      baseUI.renderBeforeOnScreenUI();
      baseUI.renderEnd();
   }

   @Override
   public void dispose()
   {
      baseUI.dispose();
   }

   private SimulationSession buildSession()
   {
      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngine::new);

      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();

      bulletPhysicsEngine.getGlobalBulletMultiBodyJointParameters().setJointDisableParentCollision(false);

      DoorDefinition doorDefinition = new DoorDefinition();
      doorDefinition.getDoorPanelDefinition().setAddArUcoMarkers(true);
      doorDefinition.build();

      // Pull door
      //  doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(1.0, -0.5, 0.01));

      // Push door
      doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(Math.PI, 0.0, 0.0), new Point3D(1.3, 0.5, 0.01));

      Robot doorRobot = simulationSession.addRobot(doorDefinition);
      DoorDefinition.setupPhysics(doorRobot, simulationSession);
      doorWidgets.initialize(doorRobot);

      simulationSession.addTerrainObject(new FlatGroundDefinition());

      return simulationSession;
   }

   public static void main(String[] args)
   {
      new RDXSCS2DoorDemo();
   }
}
