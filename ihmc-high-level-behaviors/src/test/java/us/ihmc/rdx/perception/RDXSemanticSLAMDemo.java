package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.semantic.SemanticSLAMModule;
import us.ihmc.perception.tools.PerceptionFileTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.io.FileNotFoundException;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.Set;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class RDXSemanticSLAMDemo
{
   private final ScheduledExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(),
                                                                                                           ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);

   private static final int COUNT_SKIP = 100;
   private static final int FRAME_SKIP = 1;
   private static final String LEFT_CAMERA_NAME = "image_0/";
   private static final String RIGHT_CAMERA_NAME = "image_1/";

   private static final String DATASET_PATH = System.getProperty("user.home") + "/Workspace/Data/Datasets/sequences/00/";
   private static final String GROUND_TRUTH_PATH = System.getProperty("user.home") + "/Workspace/Data/Datasets/poses/";

   //private static final String DATASET_PATH = System.getProperty("user.home") + "/Downloads/00/";
   //private static final String GROUND_TRUTH_PATH = System.getProperty("user.home") + "/Downloads/dataset/poses/";

   private boolean active = true;
   private int updateCount = 0;
   private int fileIndex = 0;
   private final Scanner gtPoseReader;
   private String fileName = "000000.png";
   private String leftImageName;
   private String rightImageName;

   private ScheduledFuture<?> scheduledFuture = null;

   private final ArrayList<ModelInstance> groundTruthPoseModels = new ArrayList<>();
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();
   private final RDXPanel panel = new RDXPanel("Visual SLAM");
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private ModelInstance modelInstance;
   private ModelInstance gtModelInstance;
   private ModelInstance landmarksLineMesh;
   private SemanticSLAMModule semanticSLAM;
   private Mat currentImageRight;
   private Mat currentImageLeft;

   private PoseListMessage poseListMessage = new PoseListMessage();

   private boolean initialized = false;

   private final Notification posesToRenderNotification = new Notification();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ArrayList<RigidBodyTransform> gtSensorTransforms = new ArrayList<>();

   private ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "semantic_slam_demo");
   private IHMCROS2Publisher<ImageMessage> posePublisher;

   public RDXSemanticSLAMDemo() throws FileNotFoundException
   {
      posePublisher = ROS2Tools.createPublisher(ros2Node, PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT), ROS2QosProfile.DEFAULT());

      gtPoseReader = new Scanner(new File(GROUND_TRUTH_PATH + "00.txt"));
      getGroundTruthPose();

      semanticSLAM = new SemanticSLAMModule();
      panel.setRenderMethod(this::renderImGuiWidgets);

      baseUI.getImGuiPanelManager().addPanel(panel);
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {

            baseUI.create();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            if (posesToRenderNotification.poll())
            {
               renderPoses(fileIndex, initialized);
            }

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            executor.shutdown();
            baseUI.dispose();
         }
      });
   }

   public void update()
   {
      if (active)
      {
         fileName = String.format("%1$6s", fileIndex).replace(' ', '0') + ".png";
         leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + fileName;
         rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + fileName;

         currentImageLeft = PerceptionFileTools.loadImage(leftImageName);
         currentImageRight = PerceptionFileTools.loadImage(rightImageName);

         Instant now = Instant.now();

         //posePublisher.publish(new ImageMessage());

         //ImageMessage imageMessage = new ImageMessage();
         //BytePointer compressedColorPointer = new BytePointer();
         //FramePose3D pose = new FramePose3D();
         //
         //PerceptionMessageTools.publishCompressedDepthImage(compressedColorPointer,
         //                                                      PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT),
         //                                                      imageMessage,
         //                                                      ros2Helper,
         //                                                      pose,
         //                                                      now,
         //                                                      0,
         //                                                      currentImageLeft.rows(),
         //                                                      currentImageLeft.cols(),
         //                                                      1.0f);

         //LogTools.info("Image Message: {}", imageMessage.getAcquisitionTime());

         //LogTools.warn("Pose: {}", now);

         initialized = semanticSLAM.update(currentImageLeft, currentImageRight);

         //posesToRenderNotification.set();
         //renderPoses(fileIndex, initialized);

         fileIndex += FRAME_SKIP;

         updateCount++;
      }
   }

   public void buildLandmarkLineMesh()
   {

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      Set<Integer> landmarkIDs = semanticSLAM.getLandmarkKeys();
      ArrayList<Point3D> points = semanticSLAM.getLandmarkPoints(landmarkIDs);

      for (Point3D point : points)
      {
         RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
         meshBuilder.addLine(0, 0, 0, point.getX(), point.getY(), point.getZ(), 0.005, Color.WHITE);
         Mesh mesh = meshBuilder.generateMesh();

         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(Color.WHITE));
         modelBuilder.part(meshPart, material);
      }

      Model model = modelBuilder.end();
      landmarksLineMesh = new ModelInstance(model);
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Next"))
      {
         update();
      }

      if (ImGui.button("Start"))
      {
         active = true;

         if (scheduledFuture == null)
         {
            scheduledFuture = executor.scheduleAtFixedRate(this::update, 0, 20L, TimeUnit.MILLISECONDS);
         }
      }

      if (ImGui.button("Pause"))
      {
         active = false;
      }
   }

   public void getGroundTruthPose()
   {
      while (gtPoseReader.hasNextLine())
      {
         String[] gtPoseSplit = gtPoseReader.nextLine().split(" ");

         //LogTools.info("GT Pose: {}", Arrays.toString(gtPoseSplit));

         // r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
         RigidBodyTransform gtTransform = new RigidBodyTransform();
         gtTransform.setUnsafe(Double.parseDouble(gtPoseSplit[0]),
                               Double.parseDouble(gtPoseSplit[1]),
                               Double.parseDouble(gtPoseSplit[2]),
                               Double.parseDouble(gtPoseSplit[3]),
                               Double.parseDouble(gtPoseSplit[4]),
                               Double.parseDouble(gtPoseSplit[5]),
                               Double.parseDouble(gtPoseSplit[6]),
                               Double.parseDouble(gtPoseSplit[7]),
                               Double.parseDouble(gtPoseSplit[8]),
                               Double.parseDouble(gtPoseSplit[9]),
                               Double.parseDouble(gtPoseSplit[10]),
                               Double.parseDouble(gtPoseSplit[11]));

         gtSensorTransforms.add(gtTransform);
      }

      LogTools.info("Total Ground Truth Transforms Loaded: {}", gtSensorTransforms.size());
   }

   public void renderPoses(int index, boolean initialized)
   {
      FramePose3D gtFramePose = new FramePose3D();
      gtFramePose.set(gtSensorTransforms.get(index));
      gtFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      ////LogTools.info("Optimized Sensor Pose: \n{}\n", gtFramePose);
      gtModelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.4, Color.GREEN);
      LibGDXTools.toLibGDX(gtFramePose, tempTransform, gtModelInstance.transform);
      gtModelInstance.transform.val[Matrix4.M03] *= 0.1;
      gtModelInstance.transform.val[Matrix4.M13] *= 0.1;
      gtModelInstance.transform.val[Matrix4.M23] *= 0.1;
      groundTruthPoseModels.add(gtModelInstance);

      if (initialized)
      {
         /* For Visualization Only */
         poseModels.clear();
         for (int i = 0; i < fileIndex / FRAME_SKIP; i++)
         {
            FramePose3D framePose = semanticSLAM.getSensorPose(i);
            framePose.changeFrame(ReferenceFrame.getWorldFrame());

            ////LogTools.info("Optimized Sensor Pose: \n{}\n", framePose);
            modelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.4, Color.CYAN);
            LibGDXTools.toLibGDX(framePose, tempTransform, modelInstance.transform);
            modelInstance.transform.val[Matrix4.M03] *= 0.1;
            modelInstance.transform.val[Matrix4.M13] *= 0.1;
            modelInstance.transform.val[Matrix4.M23] *= 0.1;
            poseModels.add(modelInstance);
         }

         semanticSLAM.clearISAM2();
      }
      LogTools.info("Total Model Instances: {}", poseModels.size());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance model : poseModels)
      {
         model.getRenderables(renderables, pool);
      }

      for (ModelInstance pose : groundTruthPoseModels)
      {
         pose.getRenderables(renderables, pool);
      }
      //landmarksLineMesh.getRenderables(renderables, pool);
   }

   public static void main(String[] args) throws FileNotFoundException
   {
      new RDXSemanticSLAMDemo();
   }
}
