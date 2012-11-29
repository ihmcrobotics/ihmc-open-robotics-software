package us.ihmc.graphics3DAdapter.graphics.instructions;

import javax.vecmath.Vector3d;

public class LinkGraphicsScale implements LinkGraphicsPrimitiveInstruction
{
   private Vector3d scaleFactor;

   public LinkGraphicsScale(double scale)
   {
      scaleFactor = new Vector3d(scale, scale, scale);
   }

   public LinkGraphicsScale(Vector3d scale)
   {
      scaleFactor = scale;
   }

   public Vector3d getScaleFactor()
   {
      return scaleFactor;
   }

   public String toString()
   {

      return "\t\t\t<Scale>"+scaleFactor+"</Scale>\n";
   }

   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
