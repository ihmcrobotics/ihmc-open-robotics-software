package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.behaviors.simulation.FlatGroundDefinition;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorDefinition;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSixDoFJoint;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * An SCS 2 simulation of a door hinged on a frame with a lever handle.
 */
public class RDXSCS2DoorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble hingeTorque = new ImDouble();
   private RDXSCS2SimulationSession rdxSimulationSession;
   private RDXSelectablePose3DGizmo doorRootPoseGizmo;
   private SimSixDoFJoint doorRootJoint;
   private SimRevoluteJoint doorHingeJoint;
   private LinkedYoDouble doorX;

   public RDXSCS2DoorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);
            baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.7, 0.0, 0.4));
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, -4.0, 4.0);

            rdxSimulationSession = new RDXSCS2SimulationSession();
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

            doorRootJoint = (SimSixDoFJoint) doorRobot.getJoint("doorRootJoint");
            doorRootPoseGizmo = new RDXSelectablePose3DGizmo();
            doorRootPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            doorHingeJoint = (SimRevoluteJoint) doorRobot.getJoint("doorHingeJoint");

            doorRobot.addController(() -> doorHingeJoint.setTau(hingeTorque.get()));

            simulationSession.addTerrainObject(new FlatGroundDefinition());

            rdxSimulationSession.getOnSessionStartedRunnables().add(() ->
            {
               rdxSimulationSession.getAdditionalImGuiWidgets().add(() ->
               {
                  ImGui.checkbox(labels.get("Move root joint"), doorRootPoseGizmo.getSelected());
                  ImGuiTools.sliderDouble(labels.get("Hinge effort"), hingeTorque, -100.0, 100.0);
               });

               YoDouble q_x = (YoDouble) rdxSimulationSession.getYoManager().getRootRegistry().findVariable("root.door.q_doorRootJoint_x");
               doorX = (LinkedYoDouble) rdxSimulationSession.getYoManager().newLinkedYoVariable(q_x);
            });

            rdxSimulationSession.create(baseUI);
            rdxSimulationSession.startSession(simulationSession);
            rdxSimulationSession.getSession().setSessionMode(SessionMode.RUNNING);
            baseUI.getPrimaryScene().addRenderableProvider(rdxSimulationSession::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(rdxSimulationSession.getControlPanel());
         }

         private void afterPhysics(double time)
         {
            if (doorRootPoseGizmo.isSelected())
            {
               doorRootJoint.getJointPose().set(doorRootPoseGizmo.getPoseGizmo().getTransformToParent());
            }
            else
            {
               doorRootPoseGizmo.getPoseGizmo().getTransformToParent().set(doorRootJoint.getJointPose());
            }
         }

         @Override
         public void render()
         {
            rdxSimulationSession.update();

            if (doorX != null)
            {
               doorX.pull();
               if (doorRootPoseGizmo.isSelected())
               {
                  doorX.getLinkedYoVariable().set(doorRootPoseGizmo.getPoseGizmo().getPose().getX());
               }
               else
               {
                  doorRootPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().setX(doorX.getLinkedYoVariable().getValue());
               }
               doorX.push();
            }

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
