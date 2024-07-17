package us.ihmc.rdx.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.objects.RDXLabFloorObject;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXPhysicsEnvironmentDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXEnvironmentBuilder environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());

   public RDXPhysicsEnvironmentDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.GROUND_TRUTH, RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);

            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            RDXLabFloorObject labFloorObject = new RDXLabFloorObject();
            labFloorObject.setPositionInWorld(new Point3D(0.0, 0.0, -0.5));
            environmentBuilder.addObject(labFloorObject);
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
