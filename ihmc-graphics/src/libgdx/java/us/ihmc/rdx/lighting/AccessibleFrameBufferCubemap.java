package us.ihmc.rdx.lighting;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Cubemap;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.FrameBufferCubemap;

/**
 * This class only exists so we can directly access bindSide(), and should never be used for any other reason
 */
public class AccessibleFrameBufferCubemap extends FrameBufferCubemap
{
   protected AccessibleFrameBufferCubemap(Pixmap.Format format, int width, int height, boolean hasDepth)
   {
      super(format, width, height, hasDepth);
   }

   protected AccessibleFrameBufferCubemap(Pixmap.Format format, int width, int height, boolean hasDepth, boolean hasStencil)
   {
      super(format, width, height, hasDepth, hasStencil);
   }

   @Override
   protected void bindSide(Cubemap.CubemapSide side)
   {
      super.bindSide(side);
   }

   protected void bindSide(Cubemap.CubemapSide side, Camera camera)
   {
      switch (side)
      {
         case NegativeX:
            camera.up.set(0, -1, 0);
            camera.direction.set(-1, 0, 0);
            break;
         case NegativeY:
            camera.up.set(0, 0, -1);
            camera.direction.set(0, -1, 0);
            break;
         case NegativeZ:
            camera.up.set(0, -1, 0);
            camera.direction.set(0, 0, -1);
            break;
         case PositiveX:
            camera.up.set(0, -1, 0);
            camera.direction.set(1, 0, 0);
            break;
         case PositiveY:
            camera.up.set(0, 0, 1);
            camera.direction.set(0, 1, 0);
            break;
         case PositiveZ:
            camera.up.set(0, -1, 0);
            camera.direction.set(0, 0, 1);
            break;
         default:
            break;
      }
      camera.update();
      bindSide(side);
   }
}
