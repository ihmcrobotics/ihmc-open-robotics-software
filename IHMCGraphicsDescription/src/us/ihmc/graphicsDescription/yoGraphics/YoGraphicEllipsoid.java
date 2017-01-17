package us.ihmc.graphicsDescription.yoGraphics;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;

public class YoGraphicEllipsoid extends YoGraphicCoordinateSystem
{
   private Vector3d radii = new Vector3d();
   private Matrix3d reshapingMatrix = new Matrix3d();
   private RigidBodyTransform reshapingTransform = new RigidBodyTransform();
   private final AppearanceDefinition appearance;

   public YoGraphicEllipsoid(String name, YoFramePoint framePoint, YoFrameOrientation orientation, AppearanceDefinition appearance, Vector3d radii)
   {
      super(name, framePoint, orientation, 1.0);
      this.appearance = appearance;
      this.radii.set(radii);
   }

   public YoGraphicEllipsoid(String namePrefix, String nameSuffix, YoVariableRegistry registry, AppearanceDefinition appearance, Vector3d radii)
   {
      super(namePrefix, nameSuffix, registry, 1.0, appearance);
      this.appearance = appearance;
      this.radii.set(radii);
   }

   public void setRadii(Vector3d radii)
   {
      this.radii.set(radii);
   }

   @Override
   protected void computeRotationTranslation(Transform3d transform3D)
   {
      reshapingMatrix.setIdentity();
      reshapingMatrix.setM00(reshapingMatrix.getM00() * radii.getX());
      reshapingMatrix.setM11(reshapingMatrix.getM11() * radii.getY());
      reshapingMatrix.setM22(reshapingMatrix.getM22() * radii.getZ());
      reshapingTransform.setRotationAndZeroTranslation(reshapingMatrix);

      transform3D.setIdentity();
      translationVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());

      transform3D.setScale(scale);
      transform3D.setRotationEulerAndZeroTranslation(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
      transform3D.setTranslation(translationVector);

      transform3D.multiply(reshapingTransform);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addEllipsoid(1.0, 1.0, 1.0, appearance); // These need to be 1.0 @dcalvert
      return linkGraphics;
   }
}
