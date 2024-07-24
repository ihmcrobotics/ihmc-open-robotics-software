package us.ihmc.rdx.lighting;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector3;
import net.mgsx.gltf.scene3d.lights.DirectionalLightEx;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXDirectionalLight
{
   private final DirectionalLightEx directionalLightEx = new DirectionalLightEx();

   public RDXDirectionalLight()
   {
      float directionalLightIntensity = 5.0f;
      Vector3 direction = new Vector3(-1.0f, -4.0f, -2.0f);
      directionalLightEx.set(Color.WHITE, direction, directionalLightIntensity);
   }

   public void setDirection(Tuple3DReadOnly direction)
   {
      LibGDXTools.toLibGDX(direction, directionalLightEx.direction);
   }

   public DirectionalLightEx getDirectionalLightEx()
   {
      return directionalLightEx;
   }
}
