package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.RealsenseColorDepthImagePublisher;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class RDXSensorTestingUI
{
   private static final int NUMBER_OF_SAMPLES = 100;
   private static final Path RESULT_FILE_DIRECTORY = Path.of(System.getProperty("user.home") + File.separator + "Documents");

   private static final String REALSENSE_SERIAL_NUMBER = "215122253249";
   private static final String REALSENSE_NAME = "D455";
   private static final String ZED_NAME = "ZED 2";

   private final RDXBaseUI baseUI = new RDXBaseUI("SensorTestingUI");
   private final RDXPerceptionVisualizersPanel visualizers = new RDXPerceptionVisualizersPanel();

   private final ZEDColorDepthImageRetriever zedImageRetriever = new ZEDColorDepthImageRetriever(0, ReferenceFrame::getWorldFrame, () -> true, () -> true);
   private final ZEDColorDepthImagePublisher zedImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES,
                                                                                                 PerceptionAPI.ZED2_DEPTH,
                                                                                                 PerceptionAPI.ZED2_CUT_OUT_DEPTH);
   private final RDXPointCloudRenderer zedPointCloudRenderer = new RDXPointCloudRenderer();
   private List<Point3D32> zedCrossSectionPointCloud;

   private final RealsenseDeviceManager realsenseManager = new RealsenseDeviceManager();
   private final RealsenseColorDepthImageRetriever realsenseImageRetriever = new RealsenseColorDepthImageRetriever(realsenseManager,
                                                                                                                   REALSENSE_SERIAL_NUMBER,
                                                                                                                   RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                                   () -> true);
   private final RealsenseColorDepthImagePublisher realsenseImagePublisher = new RealsenseColorDepthImagePublisher(PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                                                   PerceptionAPI.D455_COLOR_IMAGE);
   private final RDXPointCloudRenderer realsensePointCloudRenderer = new RDXPointCloudRenderer();
   private List<Point3D32> realsenseCrossSectionPointCloud;

   private final TypedNotification<RawImage> zedImageAvailable = new TypedNotification<>();
   private final TypedNotification<RawImage> realsenseImageAvailable = new TypedNotification<>();

   private final ImBoolean saveResultsToFile = new ImBoolean(true);
   private final ImFloat pointScale = new ImFloat(0.00025f);
   private final ImBoolean showRealsenseCrossSection = new ImBoolean(false);
   private final ImBoolean showZEDCrossSection = new ImBoolean(false);

   private boolean done = false;

   public RDXSensorTestingUI()
   {
      ThreadTools.startAThread(this::run, "SensorTesting");
      runUI();
   }

   private void run()
   {
      while (!done)
      {
         if (zedImageRetriever.isRunning())
         {
            RawImage zedDepth = zedImageRetriever.getLatestRawDepthImage();
            RawImage zedColor = zedImageRetriever.getLatestRawColorImage(RobotSide.LEFT);

            zedImageAvailable.set(zedDepth.get());

            zedImagePublisher.setNextGpuDepthImage(zedDepth.get());
            zedImagePublisher.setNextColorImage(zedColor.get(), RobotSide.LEFT);

            zedColor.release();
            zedDepth.release();
         }
         if (realsenseImageRetriever.isRunning())
         {
            RawImage realsenseDepth = realsenseImageRetriever.getLatestRawDepthImage();
            RawImage realsenseColor = realsenseImageRetriever.getLatestRawColorImage();

            realsenseImageAvailable.set(realsenseDepth.get());

            realsenseImagePublisher.setNextColorImage(realsenseColor.get());
            realsenseImagePublisher.setNextDepthImage(realsenseDepth.get());

            realsenseDepth.release();
            realsenseColor.release();
         }
      }
   }

   private RawImage getZEDDepth()
   {
      return zedImageAvailable.blockingPoll();
   }

   private RawImage getRealsenseDepth()
   {
      return realsenseImageAvailable.blockingPoll();
   }

   private void testRealsense()
   {
      ThreadTools.startAThread(() ->
      {
         SensorTester tester = new SensorTester(REALSENSE_NAME, this::getRealsenseDepth, NUMBER_OF_SAMPLES);
         tester.run();
         realsenseCrossSectionPointCloud = tester.getCrossSectionPointCloud();
         showRealsenseCrossSection.set(true);

         if (saveResultsToFile.get())
         {
            try
            {
               tester.saveDataToFile(RESULT_FILE_DIRECTORY);
            }
            catch (IOException e)
            {
               LogTools.error("Failed to save {} results to a file", REALSENSE_NAME);
            }
         }

         tester.destroy();
      }, "RealsenseTest");
   }

   private void testZED()
   {
      ThreadTools.startAThread(() ->
      {
         SensorTester tester = new SensorTester(ZED_NAME, this::getZEDDepth, NUMBER_OF_SAMPLES);
         tester.run();
         zedCrossSectionPointCloud = tester.getCrossSectionPointCloud();
         showZEDCrossSection.set(true);

         if (saveResultsToFile.get())
         {
            try
            {
               tester.saveDataToFile(RESULT_FILE_DIRECTORY);
            }
            catch (IOException e)
            {
               LogTools.error("Failed to save {}} results to a file", ZED_NAME);
            }
         }

         tester.destroy();
      }, "ZEDTest");
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            realsensePointCloudRenderer.create(1000000);
            realsensePointCloudRenderer.setPointScale(0.005f);
            zedPointCloudRenderer.create(1000000);
            zedPointCloudRenderer.setPointScale(0.005f);

            visualizers.addVisualizer(new RDXROS2ImageMessageVisualizer(ZED_NAME + " Color",
                                                                        PubSubImplementation.FAST_RTPS,
                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT)));
            visualizers.addVisualizer(new RDXROS2ColoredPointCloudVisualizer(ZED_NAME + " Point Cloud",
                                                                             PubSubImplementation.FAST_RTPS,
                                                                             PerceptionAPI.ZED2_DEPTH,
                                                                             PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT)));
            visualizers.addVisualizer(new RDXROS2ImageMessageVisualizer(REALSENSE_NAME + " Color",
                                                                        PubSubImplementation.FAST_RTPS,
                                                                        PerceptionAPI.D455_COLOR_IMAGE));
            visualizers.addVisualizer(new RDXROS2ColoredPointCloudVisualizer(REALSENSE_NAME + " Point Cloud",
                                                                             PubSubImplementation.FAST_RTPS,
                                                                             PerceptionAPI.D455_DEPTH_IMAGE,
                                                                             PerceptionAPI.D455_COLOR_IMAGE));
            visualizers.create();
            baseUI.create();
            baseUI.getImGuiPanelManager().addPanel(visualizers);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSetting);
            baseUI.getPrimaryScene().addRenderableProvider(visualizers);
            baseUI.getPrimaryScene().addRenderableProvider(realsensePointCloudRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(zedPointCloudRenderer);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            visualizers.update();

            if (showZEDCrossSection.get())
               zedPointCloudRenderer.setPointsToRender(zedCrossSectionPointCloud, Color.RED);
            else
               zedPointCloudRenderer.setPointsToRender(new ArrayList<>());

            zedPointCloudRenderer.updateMesh();

            if (showRealsenseCrossSection.get())
               realsensePointCloudRenderer.setPointsToRender(realsenseCrossSectionPointCloud, Color.BLUE);
            else
               realsensePointCloudRenderer.setPointsToRender(new ArrayList<>());

            realsensePointCloudRenderer.updateMesh();

            baseUI.renderEnd();
         }

         private void renderSetting()
         {
            ImGui.checkbox("Save results to file", saveResultsToFile);
            if (ImGui.sliderFloat("Point Scale", pointScale.getData(), 0.0f, 0.01f))
            {
               realsensePointCloudRenderer.setPointScale(pointScale.get());
               zedPointCloudRenderer.setPointScale(pointScale.get());
            }

            ImGui.separator();

            if (ImGui.button("Start " + REALSENSE_NAME))
               realsenseImageRetriever.start();
            ImGui.sameLine();
            ImGui.beginDisabled(!realsenseImageRetriever.isRunning());
            if (ImGui.button("Stop " + REALSENSE_NAME))
               realsenseImageRetriever.stop();
            if (ImGui.button("Test " + REALSENSE_NAME))
               testRealsense();
            ImGui.endDisabled();
            ImGui.checkbox("Show " + REALSENSE_NAME + " cross section", showRealsenseCrossSection);

            ImGui.separator();


            if (ImGui.button("Start " + ZED_NAME))
               zedImageRetriever.start();
            ImGui.sameLine();
            ImGui.beginDisabled(!zedImageRetriever.isRunning());
            if (ImGui.button("Stop " + ZED_NAME))
               zedImageRetriever.stop();
            if (ImGui.button("Test " + ZED_NAME))
               testZED();
            ImGui.endDisabled();
            ImGui.checkbox("Show " + ZED_NAME + " cross section", showZEDCrossSection);
         }

         @Override
         public void dispose()
         {
            done = true;

            zedImageRetriever.destroy();
            zedImagePublisher.destroy();
            realsenseImageRetriever.destroy();
            realsenseImagePublisher.destroy();

            zedPointCloudRenderer.dispose();
            realsensePointCloudRenderer.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSensorTestingUI();
   }
}
