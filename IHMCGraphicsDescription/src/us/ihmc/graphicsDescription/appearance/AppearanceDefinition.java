package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import us.ihmc.robotics.dataStructures.MutableColor;

public interface AppearanceDefinition
{
   public void setTransparency(double transparancy);

   public double getTransparency();
   
   public MutableColor getColor();
   
   public Color getAwtColor();
}
