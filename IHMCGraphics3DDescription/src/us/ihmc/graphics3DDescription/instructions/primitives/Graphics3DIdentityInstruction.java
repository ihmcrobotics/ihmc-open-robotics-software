package us.ihmc.graphics3DDescription.instructions.primitives;

import us.ihmc.graphics3DDescription.instructions.Graphics3DPrimitiveInstruction;

public class Graphics3DIdentityInstruction implements Graphics3DPrimitiveInstruction
{
   public String toString()
   {
      return "\t\t\t<Identity>\n";
   }

   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
