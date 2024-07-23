package us.ihmc.rdx.lighting;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector3;
import net.mgsx.gltf.scene3d.lights.PointLightEx;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXPointLight
{
   private final PointLightEx pointLightEx = new PointLightEx();

   public RDXPointLight()
   {
      float pointLightIntensity = 660.0f;
      Float range = null; // infinite range
      Vector3 position = new Vector3(0.0f, 0.0f, 0.0f);
      pointLightEx.set(Color.WHITE, position, pointLightIntensity, range);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      LibGDXTools.toLibGDX(position, pointLightEx.position);
   }

   public PointLightEx getPointLightEx()
   {
      return pointLightEx;
   }
}
