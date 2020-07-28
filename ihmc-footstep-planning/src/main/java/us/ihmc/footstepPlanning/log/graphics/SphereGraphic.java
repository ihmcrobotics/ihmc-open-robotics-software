package us.ihmc.footstepPlanning.log.graphics;

import javafx.scene.paint.Color;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SphereGraphic
{
   private final YoFramePoint3D center;
   private final YoDouble radius;
   private final Color color;

   public SphereGraphic(String namePrefix, Color color, YoRegistry registry)
   {
      this.center = new YoFramePoint3D(namePrefix + "_Center", ReferenceFrame.getWorldFrame(), registry);
      this.radius = new YoDouble(namePrefix + "_Radius", registry);
      this.color = color;
   }

   public YoFramePoint3D getCenter()
   {
      return center;
   }

   public YoDouble getRadius()
   {
      return radius;
   }

   public Color getColor()
   {
      return color;
   }
}
