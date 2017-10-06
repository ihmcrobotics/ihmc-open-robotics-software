package us.ihmc.graphicsDescription.instructions.primitives;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;

public class Graphics3DRotateInstruction implements Graphics3DPrimitiveInstruction
{
   private RotationMatrix rot;
   public Graphics3DRotateInstruction(RotationMatrix rot)
   {
      this.rot = rot;
   }
   public RotationMatrix getRotationMatrix()
   {
      return rot;
   }


   public String toString()
   {
      String matrix = rot.toString().replaceAll("\n", ", ");
      return "\t\t\t<RotateMatrix>"+matrix.substring(0, matrix.length() - 2)+"</RotateMatrix>\n";
   }
   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
