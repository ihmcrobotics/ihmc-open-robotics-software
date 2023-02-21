package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXRealisticSphericalDepthSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

import java.nio.ByteBuffer;

public class RDXRealisticSphericalDepthSimulationDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   private RDXRealisticSphericalDepthSimulator sphericalDepthSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private ModelInstance mousePickSphere;
   private RDXBytedecoImagePanel mainViewDepthPanel;
   private BytedecoImage image;

   public RDXRealisticSphericalDepthSimulationDemo()
   {
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

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());

            sphericalDepthSimulator = new RDXRealisticSphericalDepthSimulator("Ouster", Math.PI/2, 2048, 128,
                                                                              0.3f, 100.0f);
         }

         @Override
         public void render()
         {
            sphericalDepthSimulator.render(baseUI.getPrimaryScene());

            int aliasedRenderedAreaWidth = 2048;
            int aliasedRenderedAreaHeight = 128;

            ByteBuffer depthBuffer = sphericalDepthSimulator.getMetersDepthFloatBuffer();
            if (depthBuffer != null)
            {
               if (image == null)
               {
                  image = new BytedecoImage((int) baseUI.getPrimary3DPanel().getRenderSizeX(),
                                            (int) baseUI.getPrimary3DPanel().getRenderSizeY(),
                                            opencv_core.CV_16UC1,
                                            depthBuffer);
                  mainViewDepthPanel = new RDXBytedecoImagePanel("Main view depth",
                                                                 aliasedRenderedAreaWidth,
                                                                 aliasedRenderedAreaHeight,
                                                                 true);
                  baseUI.getImGuiPanelManager().addPanel(mainViewDepthPanel.getImagePanel());

                  baseUI.getLayoutManager().reloadLayout();
               }

               image.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null, depthBuffer);
               mainViewDepthPanel.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null);
               mainViewDepthPanel.drawDepthImage(image.getBytedecoOpenCVMat());
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            sphericalDepthSimulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRealisticSphericalDepthSimulationDemo();
   }
}
