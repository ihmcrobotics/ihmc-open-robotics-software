package us.ihmc.rdx.perception;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;

public class RDXRapidHeightMapSimulationDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ROS2Helper ros2Helper;
   private final ROS2Node ros2Node;

   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXHighLevelDepthSensorSimulator steppingL515Simulator;
   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private HumanoidPerceptionModule humanoidPerception;
   private RDXEnvironmentBuilder environmentBuilder;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;

   private final RigidBodyTransform sensorToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform sensorToGroundTransform = new RigidBodyTransform();
   private final RigidBodyTransform groundToWorldTransform = new RigidBodyTransform();
   private final Pose3D cameraPose = new Pose3D();
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("l515ReferenceFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame cameraZUpFrame = new PoseReferenceFrame("CameraZUpFrame", cameraFrame);

   private boolean initialized = false;

   public RDXRapidHeightMapSimulationDemo()
   {
      ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "height_map_simulation_ui");
      ros2Helper = new ROS2Helper(ros2Node);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DemoPullDoor.json");

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.0);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            l515PoseGizmo.create(baseUI.getPrimary3DPanel());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(l515PoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(l515PoseGizmo, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(60.0));
         }

         @Override
         public void render()
         {
            if (!initialized)
            {
               openCLManager = new OpenCLManager();

               steppingL515Simulator = RDXSimulatedSensorFactory.createRealsenseL515(l515PoseGizmo.getGizmoFrame(), () -> 0L);
               baseUI.getImGuiPanelManager().addPanel(steppingL515Simulator);
               steppingL515Simulator.setSensorEnabled(true);
               steppingL515Simulator.setPublishPointCloudROS2(false);
               steppingL515Simulator.setRenderPointCloudDirectly(false);
               steppingL515Simulator.setPublishDepthImageROS1(false);
               steppingL515Simulator.setDebugCoordinateFrame(false);
               steppingL515Simulator.setRenderColorVideoDirectly(false);
               steppingL515Simulator.setRenderDepthVideoDirectly(false);
               steppingL515Simulator.setPublishColorImageROS1(false);
               steppingL515Simulator.setPublishColorImageROS2(false);
               baseUI.getPrimaryScene().addRenderableProvider(steppingL515Simulator::getRenderables);

               bytedecoDepthImage = new BytedecoImage(steppingL515Simulator.getLowLevelSimulator().getImageWidth(),
                                                      steppingL515Simulator.getLowLevelSimulator().getImageHeight(),
                                                      opencv_core.CV_16UC1);
               bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

               humanoidPerception = new HumanoidPerceptionModule(openCLManager);
               humanoidPerception.initializeRealsenseDepthImage(steppingL515Simulator.getCopyOfCameraParameters().getHeight(),
                                                                steppingL515Simulator.getCopyOfCameraParameters().getWidth());
               humanoidPerception.initializeHeightMapExtractor(steppingL515Simulator.getLowLevelSimulator().getCameraIntrinsics());

               humanoidPerceptionUI = new RDXHumanoidPerceptionUI(humanoidPerception, ros2Helper);
               humanoidPerceptionUI.initializeHeightMapVisualizer(ros2Helper);
               humanoidPerceptionUI.initializeHeightMapUI(ros2Helper);

               baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI);
               baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI.getHeightMapVisualizer());

               baseUI.getLayoutManager().reloadLayout();

               initialized = true;
            }

            steppingL515Simulator.render(baseUI.getPrimaryScene());

            Point3D position = new Point3D(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getTranslation());
            Quaternion orientation = new Quaternion(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getRotation());
            cameraPose.set(position, orientation);
            cameraFrame.setPoseAndUpdate(cameraPose);

            sensorToWorldTransform.set(orientation, position);

            groundToWorldTransform.set(sensorToWorldTransform);
            groundToWorldTransform.getRotation().setYawPitchRoll(sensorToWorldTransform.getRotation().getYaw(), 0, 0);
            groundToWorldTransform.getTranslation().setZ(0);

            sensorToGroundTransform.set(sensorToWorldTransform);
            sensorToGroundTransform.getTranslation().setX(0.0f);
            sensorToGroundTransform.getTranslation().setY(0.0f);
            sensorToGroundTransform.getRotation()
                                   .set(new Quaternion(0.0f,
                                                       sensorToGroundTransform.getRotation().getPitch(),
                                                       sensorToGroundTransform.getRotation().getRoll()));
            RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
            groundToSensorTransform.invert();

            Pose3D groundPoseInSensorFrame = new Pose3D(groundToSensorTransform);
            cameraZUpFrame.setPoseAndUpdate(groundPoseInSensorFrame);

            humanoidPerception.updateTerrain(ros2Helper,
                                             steppingL515Simulator.getLowLevelSimulator().getMetersDepthOpenCVMat(),
                                             cameraFrame,
                                             cameraZUpFrame,
                                             initialized,
                                             true);

            humanoidPerceptionUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            steppingL515Simulator.dispose();
            humanoidPerceptionUI.destroy();
            humanoidPerception.destroy();
            bytedecoDepthImage.destroy(openCLManager);
            openCLManager.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRapidHeightMapSimulationDemo();
   }
}
