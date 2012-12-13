package us.ihmc.graphics3DAdapter.graphics.instructions;

import javax.vecmath.Vector3d;

public class Graphics3DTranslateInstruction implements Graphics3DPrimitiveInstruction
{
   private Vector3d translation = new Vector3d();

   public Graphics3DTranslateInstruction(Vector3d translation)
   {
      this.translation = translation;
   }

   public Graphics3DTranslateInstruction(double tx, double ty, double tz)
   {
      translation = new Vector3d(tx, ty, tz);
   }

   public Vector3d getTranslation()
   {
      return translation;
   }


   public String toString()
   {
      return "\t\t\t<Translate>" + translation + "</Translate>\n";
   }

   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
