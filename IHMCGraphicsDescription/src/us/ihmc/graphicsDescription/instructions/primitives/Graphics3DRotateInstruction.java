package us.ihmc.graphicsDescription.instructions.primitives;

import javax.vecmath.Matrix3d;

import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;

public class Graphics3DRotateInstruction implements Graphics3DPrimitiveInstruction
{
   private Matrix3d rot;
   public Graphics3DRotateInstruction(Matrix3d rot)
   {
      this.rot = rot;
   }
   public Matrix3d getRotationMatrix()
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
