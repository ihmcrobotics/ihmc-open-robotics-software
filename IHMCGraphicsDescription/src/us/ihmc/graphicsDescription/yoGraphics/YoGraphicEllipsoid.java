package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;

public class YoGraphicEllipsoid extends YoGraphicCoordinateSystem
{
   private Vector3D radii = new Vector3D();
   private final AppearanceDefinition appearance;

   public YoGraphicEllipsoid(String name, YoFramePoint framePoint, YoFrameOrientation orientation, AppearanceDefinition appearance, Vector3D radii)
   {
      super(name, framePoint, orientation, 1.0);
      this.appearance = appearance;
      this.radii.set(radii);
   }

   public YoGraphicEllipsoid(String namePrefix, String nameSuffix, YoVariableRegistry registry, AppearanceDefinition appearance, Vector3D radii)
   {
      super(namePrefix, nameSuffix, registry, 1.0, appearance);
      this.appearance = appearance;
      this.radii.set(radii);
   }

   public void setRadii(Vector3D radii)
   {
      this.radii.set(radii);
   }

   @Override
   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      transform3D.setIdentity();
      transform3D.setScale(scale * radii.getX(), scale * radii.getY(), scale * radii.getZ());
      transform3D.setRotationEuler(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
      transform3D.setTranslation(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addEllipsoid(1.0, 1.0, 1.0, appearance); // These need to be 1.0 @dcalvert
      return linkGraphics;
   }
}
