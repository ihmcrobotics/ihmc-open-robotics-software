package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.IterativeClosestPointTools;
import us.ihmc.perception.IterativeClosestPointWorker;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.RestartableThread;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class RDXIterativeClosestPointBasicWorkerDemo
{
   private static final int MAX_ENVIRONMENT_SIZE = 1000;
   private final int environmentSize = 5000;

   private final Random random = new Random(System.nanoTime());

   private final ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "icp_worker_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(node);
   private IterativeClosestPointWorker icpWorker = new IterativeClosestPointWorker(MAX_ENVIRONMENT_SIZE, ros2Helper, random);

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private RDXReferenceFrameGraphic referenceFrameGraphic;

   private ModelInstance mousePickSphere;
   FramePoint3D pickFramePoint = new FramePoint3D();

   private final RDXPointCloudRenderer icpBoxRenderer = new RDXPointCloudRenderer();
   private List<Point3D32> objectPointCloud;
   private final RecyclingArrayList<Point3D32> icpBoxPointCloud = new RecyclingArrayList<>(Point3D32::new);

   private final RDXPointCloudRenderer environmentPointCloudRenderer = new RDXPointCloudRenderer();
   private final RDXPointCloudRenderer segmentedPointCloudRenderer = new RDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> segmentedPtCld = new RecyclingArrayList<>(Point3D32::new);

   private boolean mouseTrackingToggle = false;

   private boolean finishedCreating = false;
   private boolean firstTick = true;

   private final Pose3D shapeInputPose = new Pose3D();
   private PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.BOX;
   private final ImInt shapeIndex = new ImInt();
   private final String[] shapeValues = new String[PrimitiveRigidBodyShape.values().length];
   private final ImFloat depth = new ImFloat(0.19f);
   private final ImFloat width = new ImFloat(0.405f);
   private final ImFloat height = new ImFloat(0.31f);
   private final ImFloat xRadius = new ImFloat(0.1f);
   private final ImFloat yRadius = new ImFloat(0.1f);
   private final ImFloat zRadius = new ImFloat(0.1f);
   private final ImInt numberOfPoints = new ImInt(1000);
   private final ImFloat segmentationRadius = new ImFloat(0.6f);

   private final ImBoolean icpGuiAutoMoveEnv = new ImBoolean(true);

   // These are the actual shape pose
   private final float[] icpGuiShapeSetPostionX = {0.0f};
   private final float[] icpGuiShapeSetPostionY = {0.0f};
   private final float[] icpGuiShapeSetPostionZ = {0.0f};
   private final float[] icpGuiShapeSetYaw = {0.0f};
   private final float[] icpGuiShapeSetPitch = {0.0f};
   private final float[] icpGuiShapeSetRoll = {0.0f};

   // These are the speed at which the input shape is moving.
   private final float[] icpGuiEnvAutoMoveSpeed = {1.0f};
   private final int[] icpGuiNumICPIterations = {1};
   private double icpGuiICPRunTimeInSeconds = 0;

   private final float[] icpGuiEnvGaussianNoiseScale = {0.0f};


   public RDXIterativeClosestPointBasicWorkerDemo()
   {
      PrimitiveRigidBodyShape[] shapeArray = new PrimitiveRigidBodyShape[PrimitiveRigidBodyShape.values().length];
      Arrays.stream(PrimitiveRigidBodyShape.values()).toList().toArray(shapeArray);

      for (int i = 0; i < PrimitiveRigidBodyShape.values().length; ++i)
      {
         shapeValues[i] = shapeArray[i].name();
      }

      RestartableThread uiThread = new RestartableThread("UI", this::runUI);
      uiThread.start();
   }

   private void generatePointsAndRunICP()
   {
      if (!finishedCreating)
         return;

      moveInputShape();
      List<Point3D32> environmentPointCloud = IterativeClosestPointTools.createICPObjectPointCloud(shape,
                                                                                                   shapeInputPose,
                                                                                                   depth.get(),
                                                                                                   width.get(),
                                                                                                   height.get(),
                                                                                                   xRadius.get(),
                                                                                                   yRadius.get(),
                                                                                                   zRadius.get(),
                                                                                                   environmentSize,
                                                                                                   random);
      addNoiseToPoints(environmentPointCloud);
      environmentPointCloudRenderer.setPointsToRender(environmentPointCloud, Color.BLUE);

      if (firstTick)
      {
         icpWorker.setPoseGuess(shapeInputPose);
         firstTick = false;
      }

      icpWorker.setEnvironmentPointCloud(environmentPointCloud);

      icpWorker.setTargetPoint(pickFramePoint);
      icpWorker.useProvidedTargetPoint(mouseTrackingToggle);
      icpWorker.setSegmentSphereRadius(segmentationRadius.get());

      long startTimeNanos = System.nanoTime();
      icpWorker.runICP(icpGuiNumICPIterations[0]);
      long stopTimeNanos = System.nanoTime();
      calculateICPTime(startTimeNanos, stopTimeNanos);

      List<Point3D32> segmentedPointCloud = icpWorker.getSegmentedPointCloud();
      segmentedPtCld.clear();
      if (segmentedPointCloud != null && !segmentedPointCloud.isEmpty())
      {
         for (int i = 0; i < MAX_ENVIRONMENT_SIZE * 10 && i < segmentedPointCloud.size(); ++i)
         {
            Point3D32 newPoint = segmentedPtCld.add();
            newPoint.set(segmentedPointCloud.get(i));
         }
         segmentedPointCloudRenderer.setPointsToRender(segmentedPtCld, Color.GRAY);
      }

      referenceFrameGraphic.setPoseInWorldFrame(icpWorker.getResultPose());
   }

   private void calculateICPTime(long time1, long time2)
   {
      long timeDiffNanos = time2-time1;
      icpGuiICPRunTimeInSeconds = (timeDiffNanos/1e9)/icpGuiNumICPIterations[0];
   }

   private void moveInputShape()
   {
      long counter = baseUI.getRenderIndex();
      if (icpGuiAutoMoveEnv.get())
      {
         double x = (Math.sin(((double) counter) * 0.001 * icpGuiEnvAutoMoveSpeed[0]) * 2) + 2.5;
         double y = (Math.sin(((double) counter) * 0.0033 * icpGuiEnvAutoMoveSpeed[0]) * 1);
         double z = (Math.sin(((double) counter) * 0.004 * icpGuiEnvAutoMoveSpeed[0]) * .5) + .5;
         double roll = (Math.sin(((double) counter) * 0.002 * icpGuiEnvAutoMoveSpeed[0]) * 3.14159);
         double pitch = (Math.sin(((double) counter) * 0.0023 * icpGuiEnvAutoMoveSpeed[0]) * 3.14159);
         double yaw = (Math.sin(((double) counter) * 0.00275 * icpGuiEnvAutoMoveSpeed[0]) * 3.14159);
         icpGuiShapeSetPostionX[0] = (float) x;
         icpGuiShapeSetPostionY[0] = (float) y;
         icpGuiShapeSetPostionZ[0] = (float) z;
         icpGuiShapeSetYaw[0] = (float) yaw;
         icpGuiShapeSetPitch[0] = (float) pitch;
         icpGuiShapeSetRoll[0] = (float) roll;
         shapeInputPose.getTranslation().set(x, y, z);
         shapeInputPose.getRotation().setYawPitchRoll(yaw, pitch, roll);
      }
      else
      {
         shapeInputPose.getTranslation().set(icpGuiShapeSetPostionX[0], icpGuiShapeSetPostionY[0], icpGuiShapeSetPostionZ[0]);
         shapeInputPose.getRotation().setYawPitchRoll(icpGuiShapeSetYaw[0], icpGuiShapeSetPitch[0], icpGuiShapeSetRoll[0]);
      }
   }

   private void addNoiseToPoints(List<Point3D32> points)
   {
      for (Point3D32 envModelPoint : points)
      {
         double xNoise = icpGuiEnvGaussianNoiseScale[0] * (random.nextDouble() - 0.5);
         double yNoise = icpGuiEnvGaussianNoiseScale[0] * (random.nextDouble() - 0.5);
         double zNoise = icpGuiEnvGaussianNoiseScale[0] * (random.nextDouble() - 0.5);
         envModelPoint.add(xNoise, yNoise, zNoise);
      }
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {

         @Override
         public void create()
         {
            mousePickSphere = RDXModelBuilder.createSphere(0.03f, Color.RED);
            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculatePickPoint);

            icpBoxRenderer.create(MAX_ENVIRONMENT_SIZE);
            baseUI.getPrimaryScene().addRenderableProvider(icpBoxRenderer, RDXSceneLevel.VIRTUAL);

            segmentedPointCloudRenderer.create(MAX_ENVIRONMENT_SIZE * 10);
            environmentPointCloudRenderer.create(MAX_ENVIRONMENT_SIZE * 10);
            baseUI.getPrimaryScene().addRenderableProvider(segmentedPointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(environmentPointCloudRenderer);

            baseUI.getImGuiPanelManager().addPanel("ICP Settings", this::renderSettings);

            referenceFrameGraphic = new RDXReferenceFrameGraphic(0.3);
            baseUI.getPrimaryScene().addRenderableProvider(referenceFrameGraphic);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            perceptionVisualizerPanel.create();

            finishedCreating = true;
         }

         @Override
         public void render()
         {
            generatePointsAndRunICP();

            if (ImGui.isMouseClicked(ImGuiMouseButton.Right))
            {
               mouseTrackingToggle = !mouseTrackingToggle;
            }

            icpBoxPointCloud.clear();
            objectPointCloud = icpWorker.getObjectPointCloud();
            if (objectPointCloud != null && !objectPointCloud.isEmpty())
            {
               for (int i = 0; i < MAX_ENVIRONMENT_SIZE && i < objectPointCloud.size(); ++i)
               {
                  Point3D32 newPoint = icpBoxPointCloud.add();
                  newPoint.set(objectPointCloud.get(i));
               }
               icpBoxRenderer.setPointsToRender(icpBoxPointCloud, Color.GOLD);
            }

            icpBoxRenderer.updateMesh();

            segmentedPointCloudRenderer.updateMesh();
            environmentPointCloudRenderer.updateMesh();

            perceptionVisualizerPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }

         private void renderSettings()
         {
            if (ImGui.combo("Shape", shapeIndex, shapeValues))
            {
               shape = PrimitiveRigidBodyShape.valueOf(shapeValues[shapeIndex.get()]);
               icpWorker = new IterativeClosestPointWorker(shape,
                                                           depth.get(),
                                                           width.get(),
                                                           height.get(),
                                                           xRadius.get(),
                                                           yRadius.get(),
                                                           zRadius.get(),
                                                           MAX_ENVIRONMENT_SIZE,
                                                           new FramePose3D(ReferenceFrame.getWorldFrame(), pickFramePoint, new RotationMatrix()),
                                                           ros2Helper,
                                                           random);
            }
            ImGui.sliderFloat("Depth", depth.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("Width", width.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("Height", height.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("xRadius", xRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("yRadius", yRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("zRadius", zRadius.getData(), 0.0f, 1.0f);
            ImGui.sliderInt("Num Points", numberOfPoints.getData(), 0, 10000);

            if (ImGui.button("Apply Size"))
            {
               icpWorker.changeSize(depth.get(), width.get(), height.get(), xRadius.get(), yRadius.get(), zRadius.get(), numberOfPoints.get());
            }
            ImGui.sliderFloat("Segmentation Radisu", segmentationRadius.getData(), 0.0f, 1.0f);

            DecimalFormat df = new DecimalFormat("#.###");
            DecimalFormat tf = new DecimalFormat("#.#########");

            ImGui.text(" ");
            ImGui.text("ICP Time: " + tf.format(icpGuiICPRunTimeInSeconds));
            ImGui.text(" ");

            ImGui.text("Input Centroid: " + df.format(shapeInputPose.getX()) + " y: " + df.format(shapeInputPose.getY()) + " z: " + df.format(shapeInputPose.getZ()));
            ImGui.text("Res Centroid: " + df.format(icpWorker.getResultPose().getX()) + " y: " + df.format(icpWorker.getResultPose().getY()) + " z: " + df.format(icpWorker.getResultPose().getZ()));
            ImGui.text("diff Centroid: " + df.format(shapeInputPose.getX() - icpWorker.getResultPose().getX()) + " y: " + df.format(shapeInputPose.getY()- icpWorker.getResultPose().getY()) + " z: " + df.format(shapeInputPose.getZ()-icpWorker.getResultPose().getZ()));

            ImGui.text(" ");
            ImGui.sliderFloat("X Position", icpGuiShapeSetPostionX, -2.0f, 2.0f);
            ImGui.sliderFloat("Y Position", icpGuiShapeSetPostionY, -2.0f, 2.0f);
            ImGui.sliderFloat("Z Position", icpGuiShapeSetPostionZ, -2.0f, 2.0f);
            ImGui.text(" ");
            ImGui.sliderFloat("X Rotation", icpGuiShapeSetYaw, -3.14f, 3.14f);
            ImGui.sliderFloat("Y Rotation", icpGuiShapeSetPitch, -3.14f, 3.14f);
            ImGui.sliderFloat("Z Rotation", icpGuiShapeSetRoll, -3.14f, 3.14f);
            ImGui.text(" ");
            ImGui.sliderFloat("Env Noise", icpGuiEnvGaussianNoiseScale, 0.0f, 0.2f);
            ImGui.text(" ");
            ImGui.sliderFloat("Move Speed", icpGuiEnvAutoMoveSpeed, 0.0f, 10.0f);
            ImGui.sliderInt("# iterations / tick", icpGuiNumICPIterations,1, 10);
            ImGui.checkbox("Auto Move", icpGuiAutoMoveEnv);
         }




         private void calculatePickPoint(us.ihmc.rdx.input.ImGui3DViewInput input)
         {
            pickFramePoint.set(input.getPickPointInWorld());
            LibGDXTools.toLibGDX(pickFramePoint, mousePickSphere.transform);
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXIterativeClosestPointBasicWorkerDemo();
   }
}
