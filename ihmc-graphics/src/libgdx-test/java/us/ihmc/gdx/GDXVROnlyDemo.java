package us.ihmc.gdx;

import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.vr.GDXVRApplication;
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
            vrApplication.getScene().addDefaultLighting();
            vrApplication.getScene().addCoordinateFrame(1.0);

            vrApplication.getVRContext().addVRInputProcessor(vrContext ->
            {
               vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
               {
                  InputDigitalActionData aButton = controller.getAButtonActionData();
                  if (aButton.bChanged() && aButton.bState())
                  {
                     vrApplication.exit();
                  }
               });
            });

            vrApplication.getScene().addRenderableProvider(((renderables, pool) ->
            {
               vrApplication.getVRContext().getControllerRenderables(renderables, pool);
               vrApplication.getVRContext().getBaseStationRenderables(renderables, pool);
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
