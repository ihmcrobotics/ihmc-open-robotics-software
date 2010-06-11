package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import javax.vecmath.Matrix3d;

import com.yobotics.simulationconstructionset.gui.XMLReaderUtility;

public class LinkGraphicsRotateMatrix implements LinkGraphicsInstruction
{
   private Matrix3d rot;
   public LinkGraphicsRotateMatrix(Matrix3d rot)
   {
      this.rot = rot;
   }
   public Matrix3d getRotationMatrix()
   {
      return rot;
   }
   
   
   public String toString()
   {
   
      return "\t\t\t<RotateMatrix>"+XMLReaderUtility.matrix3DToString(rot)+"</RotateMatrix>\n";
   }
}
