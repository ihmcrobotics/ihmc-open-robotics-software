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
import imgui.ImGui;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
//import us.ihmc.log.LogTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageMat;
import us.ihmc.perception.ImageTools;
import us.ihmc.perception.VisualSLAMModule;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;
import java.util.concurrent.ScheduledExecutorService;

public class RDXVisualSLAMDemo
{
   private final ScheduledExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(),
                                                                                                           ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final VisualSLAMModule vslam;
   private final ImGuiPanel panel = new ImGuiPanel("Visual SLAM");

   private static final int COUNT_SKIP = 100;
   private static final int FRAME_SKIP = 1;
   private static final String LEFT_CAMERA_NAME = "image_2";
   private static final String RIGHT_CAMERA_NAME = "image_3";
   private static final String DATASET_PATH = "/home/bmishra/Workspace/Data/Datasets/dataset/sequences/00/";

   private ImageMat currentImageRight;
   private ImageMat currentImageLeft;

   private String leftImageName;
   private String rightImageName;
   private String fileName = "000000.png";

   private final Scanner gtPoseReader;

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();
   private final ArrayList<ModelInstance> groundTruthPoseModels = new ArrayList<>();

   private ModelInstance modelInstance;
   private ModelInstance gtModelInstance;
   private ModelInstance landmarksLineMesh;

   private ReferenceFrame frame;

   private boolean active = false;
   private int updateCount = 0;
   private int fileIndex = 0;

   public RDXVisualSLAMDemo() throws FileNotFoundException
   {
      gtPoseReader = new Scanner(new File(DATASET_PATH + "00.txt"));

      vslam = new VisualSLAMModule();
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

            if (active)
            {
               //if(updateCount % COUNT_SKIP == 0)
               {
                  update();
               }
               updateCount++;
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

   public void buildLandmarkLineMesh()
   {

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      Set<Integer> landmarkIDs = vslam.getLandmarkKeys();
      ArrayList<Point3D> points = vslam.getLandmarkPoints(landmarkIDs);

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
         //executor.scheduleAtFixedRate(this::update, 0, 20L, TimeUnit.MILLISECONDS);
      }

      if (ImGui.button("Pause"))
      {
         active = false;
      }
   }

   public void update()
   {
      getGroundTruthPose();

      fileName = String.format("%1$6s", fileIndex).replace(' ', '0') + ".png";
      leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName;
      rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + "/" + fileName;

      currentImageLeft = ImageTools.loadAsImageMat(leftImageName);
      currentImageRight = ImageTools.loadAsImageMat(rightImageName);

      boolean initialized = vslam.update(currentImageLeft, currentImageRight);

      LogTools.info("Visual SLAM Update Completed");

      fileIndex += FRAME_SKIP;

      if (initialized)
      {
         /* For Visualization Only */
         poseModels.clear();
         for (int i = 0; i < fileIndex / FRAME_SKIP; i++)
         {
            FramePose3D framePose = vslam.getSensorPose(i);
            framePose.changeFrame(ReferenceFrame.getWorldFrame());

            ////LogTools.info("Optimized Sensor Pose: \n{}\n", framePose);
            modelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.4, Color.CYAN);
            LibGDXTools.toLibGDX(framePose, tempTransform, modelInstance.transform);
            modelInstance.transform.val[Matrix4.M03] *= 0.1;
            modelInstance.transform.val[Matrix4.M13] *= 0.1;
            modelInstance.transform.val[Matrix4.M23] *= 0.1;
            poseModels.add(modelInstance);
         }

         vslam.clearISAM2();
      }
      LogTools.info("Total Model Instances: {}", poseModels.size());
   }

   public void getGroundTruthPose()
   {
      String[] gtPoseSplit = gtPoseReader.nextLine().split(" ");

      LogTools.info("GT Pose: {}", gtPoseReader.nextLine());

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

      FramePose3D gtFramePose = new FramePose3D();
      gtFramePose.set(gtTransform);
      gtFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      ////LogTools.info("Optimized Sensor Pose: \n{}\n", gtFramePose);
      gtModelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.4, Color.GREEN);
      LibGDXTools.toLibGDX(gtFramePose, tempTransform, gtModelInstance.transform);
      gtModelInstance.transform.val[Matrix4.M03] *= 0.1;
      gtModelInstance.transform.val[Matrix4.M13] *= 0.1;
      gtModelInstance.transform.val[Matrix4.M23] *= 0.1;
      groundTruthPoseModels.add(gtModelInstance);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance model : poseModels)
      {
         model.getRenderables(renderables, pool);
      }

      for(ModelInstance pose : groundTruthPoseModels)
      {
         pose.getRenderables(renderables, pool);
      }
      //landmarksLineMesh.getRenderables(renderables, pool);
   }

   public static void main(String[] args) throws FileNotFoundException
   {
      new RDXVisualSLAMDemo();
   }
}
