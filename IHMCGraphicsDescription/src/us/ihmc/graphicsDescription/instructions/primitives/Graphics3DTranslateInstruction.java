package us.ihmc.graphicsDescription.instructions.primitives;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;

public class Graphics3DTranslateInstruction implements Graphics3DPrimitiveInstruction
{
   private Vector3d translation = new Vector3d();

   public Graphics3DTranslateInstruction(Tuple3d translation)
   {
      this.translation.set(translation);
   }

   public Graphics3DTranslateInstruction(double tx, double ty, double tz)
   {
      translation.set(tx, ty, tz);
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
