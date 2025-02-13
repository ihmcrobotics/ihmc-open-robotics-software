package us.ihmc.rdx;

import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.scene.RDXSceneLevel;
import us.ihmc.rdx.vr.RDXVRApplication;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVROnlyDemo
{
   private final RDXVRApplication vrApplication = new RDXVRApplication();

   public RDXVROnlyDemo()
   {
      vrApplication.launch(new RDXLwjgl3ApplicationAdapter()
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
               vrApplication.getVRContext().getTrackersRenderables(renderables, pool);
            }), RDXSceneLevel.VIRTUAL);
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
      new RDXVROnlyDemo();
   }
}
