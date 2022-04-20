package us.ihmc.gdx.perception;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.realsense.NonRealtimeL515;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.thread.Activator;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GDXRealsenseL515UI
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private RealSenseHardwareManager realSenseHardwareManager;
   private NonRealtimeL515 l515;

   public GDXRealsenseL515UI()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  realSenseHardwareManager = new RealSenseHardwareManager(yoRegistry, yoGraphicsListRegistry);

                  l515 = realSenseHardwareManager.createNonRealtimeL515("demo_", "F1121365");
                  l515.initialize();

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               l515.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            l515.deleteDevice();
            realSenseHardwareManager.deleteContext();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXRealsenseL515UI();
   }
}
