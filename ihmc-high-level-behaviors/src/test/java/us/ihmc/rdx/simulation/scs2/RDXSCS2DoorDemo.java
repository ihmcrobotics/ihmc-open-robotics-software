package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.type.ImDouble;
import org.bytedeco.bullet.BulletCollision.btCollisionObject;
import org.bytedeco.bullet.BulletDynamics.btMultiBody;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyLinkCollider;
import org.bytedeco.bullet.global.BulletCollision;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.behaviors.simulation.FlatGroundDefinition;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorDefinition;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletRobot;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSixDoFJoint;

/**
 * An SCS 2 simulation of a door hinged on a frame with a lever handle.
 */
public class RDXSCS2DoorDemo extends Lwjgl3ApplicationAdapter
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble hingeTorque = new ImDouble();
   private RDXSCS2RestartableSimulationSession rdxSimulationSession;
   private RDXSCS2SixDoFJointGizmo doorRootGizmo;
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
            doorRootGizmo.renderMoveJointCheckbox();
            ImGuiTools.sliderDouble(labels.get("Hinge effort"), hingeTorque, -100.0, 100.0);

            if (ImGui.button("Make kinematic"))
            {
               btMultiBody btMultiBody = bulletRobot.getBulletMultiBodyRobot().getBtMultiBody();
               btMultiBodyLinkCollider baseCollider = btMultiBody.getBaseCollider();
               baseCollider.setCollisionFlags(baseCollider.getCollisionFlags() | btCollisionObject.CF_KINEMATIC_OBJECT);
//                     baseCollider.setActivationState(BulletCollision.DISABLE_DEACTIVATION);
               for (int i = 0; i < btMultiBody.getNumLinks(); i++)
               {
                  btMultiBodyLinkCollider collider = btMultiBody.getLink(i).m_collider();
                  if (!collider.isNull())
                  {
                     collider.setCollisionFlags(collider.getCollisionFlags() | btCollisionObject.CF_KINEMATIC_OBJECT);
                     collider.setActivationState(BulletCollision.DISABLE_DEACTIVATION);
                  }
               }
            }
            if (ImGui.button("Make not kinematic"))
            {
               btMultiBody btMultiBody = bulletRobot.getBulletMultiBodyRobot().getBtMultiBody();
               btMultiBodyLinkCollider baseCollider = btMultiBody.getBaseCollider();
               baseCollider.setCollisionFlags(baseCollider.getCollisionFlags() & ~btCollisionObject.CF_KINEMATIC_OBJECT);
//                     baseCollider.setActivationState(BulletCollision.ACTIVE_TAG);
               for (int i = 0; i < btMultiBody.getNumLinks(); i++)
               {
                  btMultiBodyLinkCollider collider = btMultiBody.getLink(i).m_collider();
                  if (!collider.isNull())
                  {
                     collider.setCollisionFlags(collider.getCollisionFlags() & ~btCollisionObject.CF_KINEMATIC_OBJECT);
//                           collider.setActivationState(BulletCollision.ACTIVE_TAG);
//                           collider.activate(true);
                  }
               }
            }
         });

         doorRootGizmo.linkVariables(rdxSimulationSession.getSCS2SimulationSession().getYoManager().getRootRegistry(),
                                     rdxSimulationSession.getSCS2SimulationSession().getYoManager().getLinkedYoVariableFactory());
      });

      rdxSimulationSession.buildSimulation();
      rdxSimulationSession.getSCS2SimulationSession().getSession().setSessionMode(SessionMode.RUNNING);
   }

   @Override
   public void render()
   {
      rdxSimulationSession.update();

      doorRootGizmo.update();

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
      bulletRobot = (BulletRobot) bulletPhysicsEngine.getBulletRobots().get(0);


      doorRootGizmo = new RDXSCS2SixDoFJointGizmo(baseUI.getPrimary3DPanel(),
                                                  (SimSixDoFJoint) doorRobot.getJoint("doorRootJoint"));
      doorHingeJoint = (SimRevoluteJoint) doorRobot.getJoint("doorHingeJoint");

      doorRobot.addController(() -> doorHingeJoint.setTau(hingeTorque.get()));

      simulationSession.addTerrainObject(new FlatGroundDefinition());

      return simulationSession;
   }

   public static void main(String[] args)
   {
      new RDXSCS2DoorDemo();
   }
}
