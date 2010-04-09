package com.yobotics.simulationconstructionset.robotdefinition;

import javax.vecmath.Vector3d;

public class LinkGraphicsTranslate implements LinkGraphicsInstruction
{
   private Vector3d translation = new Vector3d();

   public LinkGraphicsTranslate(Vector3d translation)
   {
      this.translation = translation;
   }

   public LinkGraphicsTranslate(double tx, double ty, double tz)
   {
      translation = new Vector3d(tx, ty, tz);
   }

   public Vector3d getTranslation()
   {
      return translation;
   }

}
