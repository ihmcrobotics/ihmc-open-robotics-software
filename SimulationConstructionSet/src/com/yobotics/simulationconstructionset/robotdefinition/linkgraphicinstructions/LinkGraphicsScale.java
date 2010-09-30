package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import javax.vecmath.Vector3d;

public class LinkGraphicsScale implements LinkGraphicsInstruction
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
}
