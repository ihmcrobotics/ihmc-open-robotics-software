package us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes;

import java.awt.Color;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * SphereVoxelShape creates N points uniformly distributed on the surface of a sphere. For each
 * point, a ray, which goes from the point on the surface of the sphere to its origin, is generated.
 * For each ray M rotations are generated by computing the orientation aligning the x-axis to the
 * ray and transforming this orientation by M rotations around the ray. This class is meant to help
 * discretizing the 3D space of orientations and also to simulate the different possibilities for
 * grasping a spherical object.
 */
public class SphereVoxelShape
{
   public enum SphereVoxelType
   {
      graspOrigin, graspAroundSphere
   };

   private final Quaternion[][] rotations;
   private final Point3D[] pointsOnSphere;
   /** Origin of the sphere in the current voxel coordinate. Should probably always be set to zero. */
   private final Point3D sphereOrigin = new Point3D();
   private final double voxelSize;

   private final int numberOfRays;
   private final int numberOfRotationsAroundRay;

   private final SphereVoxelType type;
   private final ReferenceFrame parentFrame;

   public SphereVoxelShape(ReferenceFrame parentFrame, double voxelSize, int numberOfRays, int numberOfRotationsAroundRay, SphereVoxelType type)
   {
      this.voxelSize = voxelSize;
      this.parentFrame = parentFrame;
      this.type = type;
      this.numberOfRays = numberOfRays;
      this.numberOfRotationsAroundRay = numberOfRotationsAroundRay;

      pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(sphereOrigin, voxelSize, numberOfRays);
      rotations = SpiralBasedAlgorithm.generateOrientations(numberOfRays, numberOfRotationsAroundRay);
   }

   public int getNumberOfRays()
   {
      return numberOfRays;
   }

   public int getNumberOfRotationsAroundRay()
   {
      return numberOfRotationsAroundRay;
   }

   public void getRay(Vector3D rayToPack, int rayIndex)
   {
      MathTools.checkIntervalContains(rayIndex, 0, numberOfRays - 1);

      rayToPack.sub(sphereOrigin, pointsOnSphere[rayIndex]);
      rayToPack.normalize();
   }

   public void getOrientation(FrameQuaternion orientation, int rayIndex, int rotationAroundRayIndex)
   {
      MathTools.checkIntervalContains(rayIndex, 0, numberOfRays - 1);
      MathTools.checkIntervalContains(rotationAroundRayIndex, 0, numberOfRotationsAroundRay - 1);

      orientation.setIncludingFrame(parentFrame, rotations[rayIndex][rotationAroundRayIndex]);
   }

   public void getPose(FrameVector3D translationFromVoxelOrigin, FrameQuaternion orientation, int rayIndex, int rotationAroundRayIndex)
   {
      MathTools.checkIntervalContains(rayIndex, 0, numberOfRays - 1);
      MathTools.checkIntervalContains(rotationAroundRayIndex, 0, numberOfRotationsAroundRay - 1);

      if (type == SphereVoxelType.graspAroundSphere)
         translationFromVoxelOrigin.setIncludingFrame(parentFrame, pointsOnSphere[rayIndex]);
      else
         translationFromVoxelOrigin.setToZero(parentFrame);
      orientation.setIncludingFrame(parentFrame, rotations[rayIndex][rotationAroundRayIndex]);
   }

   public Point3D[] getPointsOnSphere()
   {
      return pointsOnSphere;
   }

   @Deprecated
   public Graphics3DObject createVisualization(FramePoint3D voxelLocation, double scale, double reachabilityValue)
   {
      ReferenceFrame originalFrame = voxelLocation.getReferenceFrame();
      voxelLocation.changeFrame(ReferenceFrame.getWorldFrame());

      Graphics3DObject voxelViz = new Graphics3DObject();

      AppearanceDefinition appearance = YoAppearance.RGBColorFromHex(Color.HSBtoRGB((float) (0.7 * reachabilityValue), 1.0f, 1.0f));

      voxelViz.translate(voxelLocation.getX(), voxelLocation.getY(), voxelLocation.getZ());
      voxelViz.addSphere(scale * voxelSize / 2.0, appearance);

      voxelLocation.changeFrame(originalFrame);

      return voxelViz;
   }

   public VisualDefinition createVisual(FramePoint3DReadOnly voxelLocation, double scale, double reachabilityValue)
   {
      FramePoint3D voxelLocationLocal = new FramePoint3D(voxelLocation);
      voxelLocationLocal.changeFrame(ReferenceFrame.getWorldFrame());

      return new VisualDefinition(voxelLocationLocal,
                                  new Sphere3DDefinition(scale * voxelSize / 2.0, 32),
                                  new MaterialDefinition(ColorDefinitions.hsb(0.7 * reachabilityValue * 360.0, 1, 1)));
   }
}
