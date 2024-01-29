package us.ihmc.rdx.simulation.scs2;

import imgui.type.ImDouble;
import us.ihmc.behaviors.simulation.FlatGroundDefinition;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorDefinition;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyLinkCollider;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletRobot;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

/**
 * An SCS 2 simulation of a door hinged on a frame with a lever handle.
 */
public class RDXSCS2DoorDemo extends Lwjgl3ApplicationAdapter
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble hingeTorque = new ImDouble();
   private final ImDouble leverTorque = new ImDouble();
   private RDXSCS2RestartableSimulationSession rdxSimulationSession;
   private SimRevoluteJoint doorHingeJoint;
   private SimRevoluteJoint doorLeverJoint;
   private BulletRobot bulletRobot;

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
         ImGuiTools.sliderDouble(labels.get("Hinge torque"), hingeTorque, -100.0, 100.0);
         ImGuiTools.sliderDouble(labels.get("Lever torque"), leverTorque, -2.0, 2.0);
      });
      rdxSimulationSession.getOnSessionStartedRunnables().add(() ->
      {
         hingeTorque.set(0.0);
         leverTorque.set(0.0);
         rdxSimulationSession.getBulletPhysicsDebugger().setUpdateDebugDrawings(true);
         rdxSimulationSession.getSession().runTick();
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
      //            doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(1.0, -0.5, 0.01));

      // Push door
      doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(Math.PI, 0.0, 0.0), new Point3D(1.3, 0.5, 0.01));

      Robot doorRobot = simulationSession.addRobot(doorDefinition);
      doorDefinition.applyPDController(doorRobot);

      bulletPhysicsEngine.getGlobalBulletMultiBodyJointParameters().setJointDisableParentCollision(true);

      bulletRobot = bulletPhysicsEngine.getBulletRobots().get(0);

      // The bolt should be slippery
      BulletMultiBodyLinkCollider boltLinkCollider = bulletRobot.getBulletMultiBodyRobot().getBulletMultiBodyLinkCollider(3);
      boltLinkCollider.setFriction(0.001);

      doorHingeJoint = (SimRevoluteJoint) doorRobot.getJoint("doorHingeJoint");
      doorLeverJoint = (SimRevoluteJoint) doorRobot.getJoint("doorLeverJoint");

      doorRobot.addController(() ->
      {
         doorHingeJoint.setTau(hingeTorque.get());
         doorLeverJoint.setTau(doorLeverJoint.getTau() + leverTorque.get());
      });

      simulationSession.addTerrainObject(new FlatGroundDefinition());

      return simulationSession;
   }

   public static void main(String[] args)
   {
      new RDXSCS2DoorDemo();
   }
}
