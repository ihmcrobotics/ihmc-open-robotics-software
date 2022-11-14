package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageMat;
import us.ihmc.perception.ImageTools;
import us.ihmc.perception.VisualSLAMModule;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.utilities.ros.RosTools;

import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;

public class RDXVisualSLAMDemo
{
   private final ScheduledExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(),
                                                                                                           ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final VisualSLAMModule vslam;
   private final ImGuiPanel panel = new ImGuiPanel("Visual SLAM");

   private static final String LEFT_CAMERA_NAME = "image_0";
   private static final String RIGHT_CAMERA_NAME = "image_1";
   private static final String DATASET_PATH = "/home/quantum/Workspace/Data/Datasets/sequences/00/";

   private ImageMat currentImageRight;
   private ImageMat currentImageLeft;

   private String leftImageName;
   private String rightImageName;
   private String fileName = "000000.png";

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();
   private ModelInstance modelInstance;

   private ReferenceFrame frame;

   private int fileIndex = 0;

   public RDXVisualSLAMDemo()
   {
      vslam = new VisualSLAMModule();
      panel.setRenderMethod(this::renderImGuiWidgets);

      baseUI.getImGuiPanelManager().addPanel(panel);
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

      //executor.scheduleAtFixedRate(this::update, 0, 20L, TimeUnit.MILLISECONDS);

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

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Next"))
      {
         update();
      }
   }

   public void update()
   {
      fileName = String.format("%1$6s", fileIndex).replace(' ', '0') + ".png";
      leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName;
      rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + "/" + fileName;

      currentImageLeft = ImageTools.loadAsImageMat(leftImageName);
      currentImageRight = ImageTools.loadAsImageMat(rightImageName);

      vslam.update(currentImageLeft, currentImageRight);

      fileIndex++;

      /* For Visualization Only */
      FramePose3D framePose = vslam.getSensorPose(fileIndex);
      framePose.changeFrame(ReferenceFrame.getWorldFrame());

      LogTools.info("Optimized Sensor Pose: \n{}\n", framePose);

      modelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.1);
      LibGDXTools.toLibGDX(framePose, tempTransform, modelInstance.transform);

      poseModels.add(modelInstance);

      LogTools.info("Total Model Instances: {}", poseModels.size());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance model : poseModels)
      {
         model.getRenderables(renderables, pool);
      }
   }

   public static void main(String[] args)
   {
      new RDXVisualSLAMDemo();
   }
}
