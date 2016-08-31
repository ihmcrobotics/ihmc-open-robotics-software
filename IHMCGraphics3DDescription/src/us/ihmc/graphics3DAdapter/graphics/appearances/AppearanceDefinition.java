package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Color;

import javax.vecmath.Color3f;

public interface AppearanceDefinition
{
   public void setTransparency(double transparancy);

   public double getTransparency();
   
   public Color3f getColor();
   
   public Color getAwtColor();
}
