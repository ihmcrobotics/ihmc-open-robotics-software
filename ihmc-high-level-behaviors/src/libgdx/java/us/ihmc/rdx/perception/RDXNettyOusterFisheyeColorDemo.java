package us.ihmc.rdx.perception;

import imgui.type.ImDouble;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableBlackflyFujinon;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.tools.thread.Activator;

public class RDXNettyOusterFisheyeColorDemo
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final RDXBaseUI baseUI = new RDXBaseUI("ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources");
   private final Activator nativesLoadedActivator;
   private final RDXNettyOusterUI nettyOusterUI = new RDXNettyOusterUI();
   private RDXBlackflyReader blackflyReader;
   private RDXInteractableBlackflyFujinon interactableBlackflyFujinon;
   private volatile boolean running = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble coloringFx = new ImDouble(472.44896); // These were tuned with sliders on the benchtop
   private final ImDouble coloringFy = new ImDouble(475.51022); // by Bhavyansh and Duncan and copied here
   private final ImDouble coloringCx = new ImDouble(970.06801); // by hand.
   private final ImDouble coloringCy = new ImDouble(608.84360); // TODO: Make them stored properties

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
                  nettyOusterUI.getSensorFrame().update(transformToBlackfly ->
                  {
                     // For the benchtop sensorhead setup
                     FramePose3D ousterPose = new FramePose3D();
                     ousterPose.getPosition().set(0.225, 0.004, 0.459);
                     RotationMatrix rotationMatrix = new RotationMatrix();
                     rotationMatrix.setAndNormalize( 0.779, -0.155,  0.607,
                                                     0.189,  0.982,  0.009,
                                                     -0.598,  0.108,  0.794);
                     ousterPose.getOrientation().set(rotationMatrix);
                     ousterPose.getOrientation().appendPitchRotation(Math.toRadians(-2));

                     RigidBodyTransform transformChestToBlackflyFujinon = new RigidBodyTransform();
                     transformChestToBlackflyFujinon.setIdentity();
                     transformChestToBlackflyFujinon.getTranslation().set(0.160, -0.095, 0.419);
                     transformChestToBlackflyFujinon.getRotation().setAndNormalize( 0.986, -0.000, 0.167, 0.000, 1.000, -0.000, -0.167, 0.000, 0.986);
                     ReferenceFrame blackflyFrame
                           = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                      transformChestToBlackflyFujinon);

                     ousterPose.changeFrame(blackflyFrame);
                     ousterPose.get(transformToBlackfly);
                  });

                  blackflyReader.setMonitorPanelUIThreadPreprocessor(texture ->
                  {
                     nettyOusterUI.getDepthImageToPointCloudKernel().setFisheyeImageToColorPoints(texture.getRGBA8Image(),
                                                                                                  coloringFx.get(),
                                                                                                  coloringFy.get(),
                                                                                                  coloringCx.get(),
                                                                                                  coloringCy.get());
                     nettyOusterUI.getSensorFrame().getReferenceFrame()
                                  .getTransformToDesiredFrame(nettyOusterUI.getDepthImageToPointCloudKernel().getOusterToFisheyeTransformToPack(),
                                                              interactableBlackflyFujinon.getInteractableFrameModel().getReferenceFrame());
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

                     baseUI.getPrimaryScene().addRenderableProvider(nettyOusterUI.getPointCloudRenderer(), RDXSceneLevel.MODEL);
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