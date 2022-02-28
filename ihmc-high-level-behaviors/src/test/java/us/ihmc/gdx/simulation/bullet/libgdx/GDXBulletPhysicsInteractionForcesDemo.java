package us.ihmc.gdx.simulation.bullet.libgdx;

import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.simulation.bullet.GDXBulletTools;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.objects.GDXLabFloorObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXBulletPhysicsInteractionForcesDemo
{

   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXEnvironmentBuilder environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
   private final ImFloat blockTransparency = new ImFloat(1.0f);

   public GDXBulletPhysicsInteractionForcesDemo()
   {
      GDXBulletTools.ensureBulletInitialized();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {

         private GDXMediumCinderBlockRoughed mediumCinderBlockRoughed;

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.get3DSceneManager().addCoordinateFrame(0.3);

            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            GDXLabFloorObject labFloorObject = new GDXLabFloorObject();
            environmentBuilder.addObject(labFloorObject);
            labFloorObject.copyThisTransformToBulletMultiBody();

            recreateAndPlace();

            ImGuiPanel experimentPanel = new ImGuiPanel("Demo", () ->
            {
               if (ImGui.button("Replace Block"))
               {
                  recreateAndPlace();
               }
               if (ImGui.sliderFloat("Block transparency", blockTransparency.getData(), 0.0f, 1.0f))
               {
                  GDXTools.setTransparency(mediumCinderBlockRoughed.getRealisticModelInstance(), blockTransparency.get());
               }
            });
            baseUI.getImGuiPanelManager().addPanel(experimentPanel);
         }

         public void recreateAndPlace()
         {
            if (mediumCinderBlockRoughed != null)
            {
               environmentBuilder.removeObject(mediumCinderBlockRoughed);
            }

            mediumCinderBlockRoughed = new GDXMediumCinderBlockRoughed();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.getTranslation().set(0.0, 0.0, 0.5);
            transformToWorld.getRotation().setYawPitchRoll(Math.toRadians(45.0), Math.toRadians(45.0), Math.toRadians(15.0));
            mediumCinderBlockRoughed.setTransformToWorld(transformToWorld);
            environmentBuilder.addObject(mediumCinderBlockRoughed);
            mediumCinderBlockRoughed.copyThisTransformToBulletMultiBody();
         }

         @Override
         public void render()
         {
            environmentBuilder.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXBulletPhysicsInteractionForcesDemo();
   }
}
