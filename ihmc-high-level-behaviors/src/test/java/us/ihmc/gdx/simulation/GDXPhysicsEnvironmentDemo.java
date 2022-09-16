package us.ihmc.gdx.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.simulation.bullet.GDXBulletTools;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.objects.GDXLabFloorObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMultiBodySnakeObject;
import us.ihmc.gdx.simulation.environment.object.objects.door.GDXDoorObject;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXPhysicsEnvironmentDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXEnvironmentBuilder environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());

   public GDXPhysicsEnvironmentDemo()
   {
      GDXBulletTools.ensureBulletInitialized();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            GDXLabFloorObject labFloorObject = new GDXLabFloorObject();
            labFloorObject.setPositionInWorld(new Point3D(0.0, 0.0, -0.5));
            environmentBuilder.addObject(labFloorObject);
            labFloorObject.copyThisTransformToBulletMultiBody();

            GDXMultiBodySnakeObject multiBodySnake = new GDXMultiBodySnakeObject();
            multiBodySnake.setPositionInWorld(new Point3D(0.0, 0.0, 4.0));
            environmentBuilder.addObject(multiBodySnake);
            multiBodySnake.copyThisTransformToBulletMultiBody();

            GDXDoorObject doorObject = new GDXDoorObject();
            doorObject.setPositionInWorld(new Point3D(0.0, 2.0, 0.0));
            environmentBuilder.addObject(doorObject);
            doorObject.copyThisTransformToBulletMultiBody();
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
      new GDXPhysicsEnvironmentDemo();
   }
}
