package us.ihmc.rdx.perception;

import imgui.type.ImDouble;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableBlackflyFujinon;

public class RDXNettyOusterFisheyeColorDemo
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXNettyOusterUI nettyOusterUI = new RDXNettyOusterUI();
   private RDXBlackflyReader blackflyReader;
   private RDXInteractableBlackflyFujinon interactableBlackflyFujinon;
   private volatile boolean running = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final IntrinsicCameraMatrixProperties ousterFisheyeColoringInstrinsics = SensorHeadParameters.loadOusterFisheyeColoringIntrinsicsBenchtop();
   private final ImDouble coloringFx = new ImDouble(ousterFisheyeColoringInstrinsics.getFocalLengthX());
   private final ImDouble coloringFy = new ImDouble(ousterFisheyeColoringInstrinsics.getFocalLengthY());
   private final ImDouble coloringCx = new ImDouble(ousterFisheyeColoringInstrinsics.getPrinciplePointX());
   private final ImDouble coloringCy = new ImDouble(ousterFisheyeColoringInstrinsics.getPrinciplePointY());

   public RDXNettyOusterFisheyeColorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            nettyOusterUI.create(baseUI);
            RDXPanel panel = new RDXPanel("Ouster Coloring", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            blackflyReader = new RDXBlackflyReader(BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());

            blackflyReader.create();
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapImagePanel().getImagePanel());

            interactableBlackflyFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel());
            interactableBlackflyFujinon.getInteractableFrameModel().setPose(SensorHeadParameters.FISHEYE_RIGHT_TO_OUSTER_TRANSFORM_ON_ROBOT);

            blackflyReader.setMonitorPanelUIThreadPreprocessor(texture ->
            {
               if (nettyOusterUI.getIsReady())
               {
                  nettyOusterUI.setFisheyeImageToColorPoints(texture.getRGBA8Image(),
                                                             coloringFx.get(),
                                                             coloringFy.get(),
                                                             coloringCx.get(),
                                                             coloringCy.get());
                  nettyOusterUI.getSensorFrame()
                               .getReferenceFrame()
                               .getTransformToDesiredFrame(nettyOusterUI.getOusterFisheyeKernel()
                                                                        .getOusterToFisheyeTransformToPack(),
                                                           interactableBlackflyFujinon.getInteractableFrameModel()
                                                                                      .getReferenceFrame());
               }
            });

            ThreadTools.startAsDaemon(() ->
            {
               while (running)
               {
                  blackflyReader.readBlackflyImage();
               }
            }, "CameraRead");
         }

         /**
          * The threading here isn't really ideal. The rendered image and point cloud
          * only need to be rendered after onFrameReceived in a new thread. But I didn't
          * find it necessary to spend the time on the thread barriers for those yet,
          * so we just run the kernels everytime and sync over the copy.
          */
         @Override
         public void render()
         {
            interactableBlackflyFujinon.update();
            blackflyReader.updateOnUIThread();

            if (nettyOusterUI.isOusterInitialized())
            {
               if (nettyOusterUI.getImagePanel() == null)
               {
                  nettyOusterUI.createAfterOusterInitialized();

                  baseUI.getPrimaryScene().addRenderableProvider(nettyOusterUI::getRenderables);
                  baseUI.getImGuiPanelManager().addPanel(nettyOusterUI.getImagePanel().getImagePanel());
                  baseUI.getLayoutManager().reloadLayout();
               }

               nettyOusterUI.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            nettyOusterUI.renderImGuiWidgets();
            ImGuiTools.sliderDouble(labels.get("Coloring Fx (px)"), coloringFx, -100.0, 800.0, "%.5f");
            ImGuiTools.sliderDouble(labels.get("Coloring Fy (px)"), coloringFy, -100.0, 800.0, "%.5f");
            ImGuiTools.sliderDouble(labels.get("Coloring Cx (px)"), coloringCx, -100.0, 1200.0, "%.5f");
            ImGuiTools.sliderDouble(labels.get("Coloring Cy (px)"), coloringCy, -1000.0, 1200.0, "%.5f");
         }

         @Override
         public void dispose()
         {
            running = false;
            nettyOusterUI.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXNettyOusterFisheyeColorDemo();
   }
}