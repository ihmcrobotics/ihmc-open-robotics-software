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
   private RDXSCS2RestartableSimulationSession rdxSimulationSession;
   private RDXSCS2BulletRobotMover doorRobotMover;
   private SimRevoluteJoint doorHingeJoint;
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

      rdxSimulationSession = new RDXSCS2RestartableSimulationSession(baseUI, this::buildSession);
      rdxSimulationSession.getOnSessionStartedRunnables().add(() ->
      {
         rdxSimulationSession.getSCS2SimulationSession().getAdditionalImGuiWidgets().add(() ->
         {
            doorRobotMover.renderMoveJointCheckbox();
            ImGuiTools.sliderDouble(labels.get("Hinge effort"), hingeTorque, -100.0, 100.0);
         });

         hingeTorque.set(0.0);
         rdxSimulationSession.getSCS2SimulationSession().getBulletPhysicsDebugger().setUpdateDebugDrawings(true);
         rdxSimulationSession.getSCS2SimulationSession().getSession().runTick();
      });

      rdxSimulationSession.buildSimulation();
   }

   @Override
   public void render()
   {
      rdxSimulationSession.update();

      if (doorRobotMover != null)
         doorRobotMover.update();

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

      DoorDefinition doorDefinition = new DoorDefinition();
      doorDefinition.getDoorPanelDefinition().setAddArUcoMarkers(true);
      doorDefinition.build();

      // Pull door
      //            doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(1.0, -0.5, 0.01));

      // Push door
      doorDefinition.getInitialSixDoFState().setConfiguration(new YawPitchRoll(Math.PI, 0.0, 0.0), new Point3D(1.3, 0.5, 0.01));
      Robot doorRobot = simulationSession.addRobot(doorDefinition);
      doorDefinition.applyPDController(doorRobot);

      BulletPhysicsEngine bulletPhysicsEngine = (BulletPhysicsEngine) simulationSession.getPhysicsEngine();
      bulletRobot = bulletPhysicsEngine.getBulletRobots().get(0);

      doorRobotMover = new RDXSCS2BulletRobotMover(baseUI.getPrimary3DPanel(), bulletRobot);
      doorHingeJoint = (SimRevoluteJoint) doorRobot.getJoint("doorHingeJoint");

      doorRobot.addController(() -> doorHingeJoint.setTau(hingeTorque.get()));

      simulationSession.addTerrainObject(new FlatGroundDefinition());
      simulationSession.addBeforePhysicsCallback(doorRobotMover::beforePhysics);

      return simulationSession;
   }

   public static void main(String[] args)
   {
      new RDXSCS2DoorDemo();
   }
}
