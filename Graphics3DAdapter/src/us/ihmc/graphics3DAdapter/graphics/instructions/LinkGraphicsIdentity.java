package us.ihmc.graphics3DAdapter.graphics.instructions;

public class LinkGraphicsIdentity implements LinkGraphicsPrimitiveInstruction
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
