package com.yobotics.simulationconstructionset.robotdefinition;

import javax.vecmath.Matrix3d;

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
}
