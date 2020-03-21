package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;

public class CustomJMECamera extends Camera
{
   public CustomJMECamera(Camera copyFromCamera)
   {
//      copyFrom(copyFromCamera);

   }

   public CustomJMECamera(int width, int height)
   {
      super(width, height);

      setFrustumPerspective(45f, (float) width / height, 1f, 1000f);
      setLocation(new Vector3f(0f, 0f, 10f));
      lookAt(new Vector3f(0f, 0f, 0f), Vector3f.UNIT_Y);

      // set clip distance near 0.001f
   }
}
