package us.ihmc.graphics3DAdapter.graphics.appearances;

import javax.vecmath.Color3f;

public interface AppearanceDefinition
{
   public void setTransparency(double transparancy);

   public double getTransparency();
   
   public Color3f getColor();
}
