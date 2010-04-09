package com.yobotics.simulationconstructionset.robotdefinition;

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
}
