package us.ihmc.yoUtilities.graphics;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;

/**
 * Last updated by: mjohnson
 * On: 7/22/11 9:57 AM
 */
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

   public void setRadii(Point3d radii)
   {
      this.radii.set(radii);
   }

   protected void computeRotationTranslation(Transform3d transform3D)
   {
      reshapingMatrix.setIdentity();
      reshapingMatrix.m00 = reshapingMatrix.m00 * radii.getX();
      reshapingMatrix.m11 = reshapingMatrix.m11 * radii.getY();
      reshapingMatrix.m22 = reshapingMatrix.m22 * radii.getZ();
      reshapingTransform.setRotationAndZeroTranslation(reshapingMatrix);

      transform3D.setIdentity();
      translationVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());

      transform3D.setScale(scale);
      transform3D.setEuler(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
      transform3D.setTranslation(translationVector);

      transform3D.multiply(reshapingTransform);
   }

   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addEllipsoid(0.5, 0.5, 0.5, appearance);
      return linkGraphics;
   }
}
