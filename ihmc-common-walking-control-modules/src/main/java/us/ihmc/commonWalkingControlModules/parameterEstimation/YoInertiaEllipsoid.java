package us.ihmc.commonWalkingControlModules.parameterEstimation;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoInertiaEllipsoid
{
   private final YoVector3D radii;
   private final YoInteger rgba;
   private final YoColorRGBASingleDefinition color;

   public YoInertiaEllipsoid(String namePrefix, YoRegistry registry)
   {
      radii = new YoVector3D(namePrefix + "Radii", registry);
      rgba = new YoInteger(namePrefix + "RGBAInteger", registry);
      color = new YoColorRGBASingleDefinition(rgba.getName());
   }

   public void setRadii(Vector3D radii)
   {
      this.radii.set(radii);
   }

   public YoVector3D getRadii()
   {
      return radii;
   }

   public void setColor(int rgba)
   {
      // The 'color' is defined by this YoInteger. We do not change 'color' directly.
      this.rgba.set(rgba);
   }

   /** Gets the RGBA YoInteger which defines the color. Change this integer to change the color. */
   public YoInteger getColorInteger()
   {
      return rgba;
   }

   public YoColorRGBASingleDefinition getColor()
   {
      return color;
   }
}
