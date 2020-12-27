package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.utils.Array;

import java.util.function.Consumer;

/**
 * Adapted from:
 * https://github.com/badlogic/gdx-vr/blob/master/src/com/badlogic/gdx/vr/VRContext.java
 */
public class GDXVRApplication
{
   private static final String TAG = GDXVRApplication.class.getSimpleName();

   private GDXVRContext context;
   private final Consumer<Camera> renderer;
   private Array<ModelInstance> modelInstances = new Array<>();

   public GDXVRApplication(Consumer<Camera> renderer)
   {
      this.renderer = renderer;
   }

   public void create()
   {
      context = new GDXVRContext();

      // Set the far clip plane distance on the camera of each eye
      // All units are in meters.
      context.getEyeData(GDXVRContext.Eye.Left).camera.far = 100f;
      context.getEyeData(GDXVRContext.Eye.Right).camera.far = 100f;

      // Register a VRDeviceListener to get notified when
      // controllers are (dis-)connected and their buttons
      // are pressed. Note that we add/remove a ModelInstance for
      // controllers for rendering on (dis-)connect.
      context.addListener(new GDXVRContext.VRDeviceListener() {
         @Override
         public void connected(GDXVRContext.VRDevice device) {
            Gdx.app.log(TAG, device + " connected");
            if (device.getType() == GDXVRContext.VRDeviceType.Controller && device.getModelInstance() != null)
               modelInstances.add(device.getModelInstance());
         }

         @Override
         public void disconnected(GDXVRContext.VRDevice device) {
            Gdx.app.log(TAG, device + " disconnected");
            if (device.getType() == GDXVRContext.VRDeviceType.Controller && device.getModelInstance() != null)
               modelInstances.removeValue(device.getModelInstance(), true);
         }

         @Override
         public void buttonPressed(GDXVRContext.VRDevice device, int button) {
            Gdx.app.log(TAG, device + " button pressed: " + button);
         }

         @Override
         public void buttonReleased(GDXVRContext.VRDevice device, int button) {
            Gdx.app.log(TAG, device + " button released: " + button);
         }
      });
   }

   public void render() {
      // poll the latest tracking data.
      // must be called before context.begin()
      context.pollEvents();

      // render the scene for the left/right eye
      context.begin();
      renderScene(GDXVRContext.Eye.Left);
      renderScene(GDXVRContext.Eye.Right);
      context.end();
   }

   private void renderScene(GDXVRContext.Eye eye) {
      GDXVRCamera camera = context.getEyeData(eye).camera;
      context.beginEye(eye);
      renderer.accept(camera);
      context.endEye();
   }

   public void dispose() {
      if (context != null)
         context.dispose();
   }

   public Array<ModelInstance> getModelInstances()
   {
      return modelInstances;
   }
}
