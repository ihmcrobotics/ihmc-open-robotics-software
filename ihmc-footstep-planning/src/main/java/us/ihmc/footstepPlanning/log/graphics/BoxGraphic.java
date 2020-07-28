package us.ihmc.footstepPlanning.log.graphics;

import javafx.scene.paint.Color;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BoxGraphic
{
   private final YoFramePose3D boxPose;
   private final YoDouble sizeX, sizeY, sizeZ;
   private final Color color;

   public BoxGraphic(String namePrefix, Color color, YoRegistry registry)
   {
      this.boxPose = new YoFramePose3D(namePrefix + "_Box", ReferenceFrame.getWorldFrame(), registry);
      this.sizeX = new YoDouble(namePrefix + "SizeX", registry);
      this.sizeY = new YoDouble(namePrefix + "SizeY", registry);
      this.sizeZ = new YoDouble(namePrefix + "SizeZ", registry);
      this.color = color;
   }

   public YoFramePose3D getBoxPose()
   {
      return boxPose;
   }

   public YoDouble getSizeX()
   {
      return sizeX;
   }

   public YoDouble getSizeY()
   {
      return sizeY;
   }

   public YoDouble getSizeZ()
   {
      return sizeZ;
   }

   public Color getColor()
   {
      return color;
   }
}
