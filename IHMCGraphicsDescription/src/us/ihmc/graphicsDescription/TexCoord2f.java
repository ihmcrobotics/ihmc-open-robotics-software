package us.ihmc.graphicsDescription;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

public class TexCoord2f implements Tuple2DBasics
{
   public float x, y;

   public TexCoord2f()
   {
   }

   public TexCoord2f(float textureX, float textureY)
   {
      x = textureX;
      y = textureY;
   }

   public TexCoord2f(TexCoord2f texCoord2f)
   {
      set(texCoord2f);
   }

   public TexCoord2f(float[] textureLocation)
   {
      set(textureLocation);
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public void setX(double x)
   {
      this.x = (float) x;
   }

   @Override
   public void setY(double y)
   {
      this.y = (float) y;
   }

   @Override
   public void applyTransform(Transform transform, boolean checkIfTransformInXYplane)
   {
      throw new UnsupportedOperationException();
   }
}
