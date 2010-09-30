package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import javax.vecmath.Vector3d;

public class LinkGraphicsRotate implements LinkGraphicsInstruction
{
   private double angle;
   private Vector3d axis;

   public LinkGraphicsRotate(double angle, Vector3d axis)
   {
      this.angle = angle;
      this.axis = axis;
   }
   public double getAngle()
   {
      return angle;
   }
   public Vector3d getAxis()
   {
      return axis;
   }

   public String toString()
   {
      return "\t\t\t<Rotate>"+angle+","+axis.x+","+axis.y+","+axis.z+"</Rotate>\n";
   }
}
