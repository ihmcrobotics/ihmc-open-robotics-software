package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.Color;
import org.bytedeco.bullet.LinearMath.btVector3;

public class BulletLibGDXTools
{
   public static void toLibGDX(btVector3 bulletColor, Color libGDXColor)
   {
      libGDXColor.set((float) bulletColor.getX(), (float) bulletColor.getY(), (float) bulletColor.getZ(), 1.0f);
   }
}
