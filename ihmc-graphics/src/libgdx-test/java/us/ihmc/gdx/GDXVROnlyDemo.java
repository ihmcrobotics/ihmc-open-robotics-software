package us.ihmc.gdx;

import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.vr.GDXVRApplication;
import us.ihmc.gdx.vr.GDXVRControllerButtons;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVROnlyDemo
{
   private final GDXVRApplication vrApplication = new GDXVRApplication();

   public GDXVROnlyDemo()
   {
      vrApplication.launch(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            vrApplication.getSceneBasics().addDefaultLighting();
            vrApplication.getSceneBasics().addCoordinateFrame(1.0);

            vrApplication.getVRContext().addVRInputProcessor(vrContext ->
            {
               vrContext.getController(RobotSide.RIGHT, controller ->
               {
                  if (controller.isButtonNewlyPressed(GDXVRControllerButtons.INDEX_A))
                  {
                     vrApplication.exit();
                  }
               });
            });

            vrApplication.getSceneBasics().addRenderableProvider(((renderables, pool) ->
            {
               for (RobotSide side : RobotSide.values)
               {
                  vrApplication.getVRContext().getController(side, controller -> controller.getModelInstance().getRenderables(renderables, pool));
                  vrApplication.getVRContext().getEyes().get(side).getCoordinateFrameInstance().getRenderables(renderables, pool);
               }
               vrApplication.getVRContext().getBaseStations(baseStation -> baseStation.getModelInstance().getRenderables(renderables, pool));
               vrApplication.getVRContext().getGenericDevices(genericDevice -> genericDevice.getModelInstance().getRenderables(renderables, pool));
            }), GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {

         }

         @Override
         public void dispose()
         {

         }
      });
   }

   public static void main(String[] args)
   {
      new GDXVROnlyDemo();
   }
}
