package us.ihmc.rdx.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.simulation.bullet.RDXBulletTools;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.objects.RDXLabFloorObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXMultiBodySnakeObject;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXDoorObject;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXPhysicsEnvironmentDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXEnvironmentBuilder environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());

   public RDXPhysicsEnvironmentDemo()
   {
      RDXBulletTools.ensureBulletInitialized();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            RDXLabFloorObject labFloorObject = new RDXLabFloorObject();
            labFloorObject.setPositionInWorld(new Point3D(0.0, 0.0, -0.5));
            environmentBuilder.addObject(labFloorObject);
            labFloorObject.copyThisTransformToBulletMultiBody();

            RDXMultiBodySnakeObject multiBodySnake = new RDXMultiBodySnakeObject();
            multiBodySnake.setPositionInWorld(new Point3D(0.0, 0.0, 4.0));
            environmentBuilder.addObject(multiBodySnake);
            multiBodySnake.copyThisTransformToBulletMultiBody();

            RDXDoorObject doorObject = new RDXDoorObject();
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
      new RDXPhysicsEnvironmentDemo();
   }
}
