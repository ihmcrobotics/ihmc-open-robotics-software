package us.ihmc.rdx.perception;

import imgui.type.ImDouble;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableBlackflyFujinon;
import us.ihmc.tools.thread.Activator;

public class RDXNettyOusterFisheyeColorDemo
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final Activator nativesLoadedActivator;
   private final RDXNettyOusterUI nettyOusterUI = new RDXNettyOusterUI();
   private RDXBlackflyReader blackflyReader;
   private RDXInteractableBlackflyFujinon interactableBlackflyFujinon;
   private volatile boolean running = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble coloringFx = new ImDouble(SensorHeadParameters.FOCAL_LENGTH_X_FOR_COLORING);
   private final ImDouble coloringFy = new ImDouble(SensorHeadParameters.FOCAL_LENGTH_Y_FOR_COLORING);
   private final ImDouble coloringCx = new ImDouble(SensorHeadParameters.PRINCIPAL_POINT_X_FOR_COLORING);
   private final ImDouble coloringCy = new ImDouble(SensorHeadParameters.PRINCIPAL_POINT_Y_FOR_COLORING);

   public RDXNettyOusterFisheyeColorDemo()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            nettyOusterUI.create(baseUI);
            ImGuiPanel panel = new ImGuiPanel("Ouster Coloring", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            blackflyReader = new RDXBlackflyReader(nativesLoadedActivator, BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());
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
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  blackflyReader.create();
                  baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapImagePanel().getImagePanel());

                  interactableBlackflyFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel());

                  nettyOusterUI.createAfterNativesLoaded();
                  nettyOusterUI.getSensorFrame().update(transformToBlackfly -> transformToBlackfly.set(SensorHeadParameters.OUSTER_TO_FISHEYE_TRANSFORM));

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
                                     .getTransformToDesiredFrame(nettyOusterUI.getOusterFisheyeKernel().getOusterToFisheyeTransformToPack(),
                                                                 interactableBlackflyFujinon.getInteractableFrameModel().getReferenceFrame());
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
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            nettyOusterUI.renderImGuiWidgets();
            ImGuiTools.sliderDouble(labels.get("Coloring Fx (px)"), coloringFx, -100.0, 800.0 , "%.5f");
            ImGuiTools.sliderDouble(labels.get("Coloring Fy (px)"), coloringFy, -100.0,  800.0 , "%.5f");
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