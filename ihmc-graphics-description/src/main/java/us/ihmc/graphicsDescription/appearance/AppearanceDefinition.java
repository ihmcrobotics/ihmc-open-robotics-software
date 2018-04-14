package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import us.ihmc.graphicsDescription.color.MutableColor;

public interface AppearanceDefinition
{
   public void setTransparency(double transparancy);

   public double getTransparency();
   
   public MutableColor getColor();
   
   public Color getAwtColor();
}
