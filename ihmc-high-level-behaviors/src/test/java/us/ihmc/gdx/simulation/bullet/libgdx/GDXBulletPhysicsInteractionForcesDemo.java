package us.ihmc.gdx.simulation.bullet.libgdx;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.simulation.bullet.GDXBulletTools;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.objects.GDXLabFloorObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXBulletPhysicsInteractionForcesDemo
{

   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXEnvironmentBuilder environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());

   public GDXBulletPhysicsInteractionForcesDemo()
   {
      GDXBulletTools.ensureBulletInitialized();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
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

            GDXMediumCinderBlockRoughed mediumCinderBlockRoughed = new GDXMediumCinderBlockRoughed();
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
