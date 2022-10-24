package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;

/**
 * This class resolves where the Center of Pressure is for a given wrench and plane. If the z
 * component of the force is close to zero, then the returned CoP will be just at the origin of the
 * passed in referenceFrame, and there still may be components of moment in the plane.
 * <p>
 * The Center of Pressure is the point at which components of moment in the plane become zero. It
 * can be viewed as the point in which the wrench could be resisted by a universal joint (freely
 * spinning bearing on the x axis and y axis of the given plane). Since the Center of Pressure
 * requires a plane, it makes sense for flat ground, or situations in which the plane is obvious.
 * However, for multi-contact, it may not make as much sense to pick a plane. The plane may be
 * arbitrary, except perhaps for a flat plane, in which the surface normal is aligned with the
 * gravity vector.
 * </p>
 */
public class CenterOfPressureResolver
{
   private final SpatialForce wrenchResolvedOnPlane = new SpatialForce();
   private final Vector3D torqueAtZeroInPlaneFrame = new Vector3D();
   private final Vector3D forceInPlaneFrame = new Vector3D();

   public double resolveCenterOfPressureAndNormalTorque(FramePoint2DBasics centerOfPressureToPack,
                                                        SpatialForceReadOnly spatialForceVector,
                                                        ReferenceFrame centerOfPressurePlaneFrame)
   {
      centerOfPressureToPack.setReferenceFrame(centerOfPressurePlaneFrame);
      return resolveCenterOfPressureAndNormalTorque((FixedFramePoint2DBasics) centerOfPressureToPack, spatialForceVector, centerOfPressurePlaneFrame);
   }

   public double resolveCenterOfPressureAndNormalTorque(FixedFramePoint2DBasics centerOfPressureToPack,
                                                        SpatialForceReadOnly spatialForceVector,
                                                        ReferenceFrame centerOfPressurePlaneFrame)
   {
      // First resolve the wrench at the plane origin:
      wrenchResolvedOnPlane.setIncludingFrame(spatialForceVector);
      wrenchResolvedOnPlane.changeFrame(centerOfPressurePlaneFrame);

      torqueAtZeroInPlaneFrame.set(wrenchResolvedOnPlane.getAngularPart());
      forceInPlaneFrame.set(wrenchResolvedOnPlane.getLinearPart());

      double fz = forceInPlaneFrame.getZ();

      double vector12x = Double.NaN;
      double vector12y = Double.NaN;

      double normalTorqueAtCenterOfPressure;
      if (fz > 1e-7)
      {
         //with sufficient normal force
         vector12x = -1.0 / fz * torqueAtZeroInPlaneFrame.getY();
         vector12y = 1.0 / fz * torqueAtZeroInPlaneFrame.getX();
         normalTorqueAtCenterOfPressure = torqueAtZeroInPlaneFrame.getZ() - vector12x * forceInPlaneFrame.getY() + vector12y * forceInPlaneFrame.getX();
      }
      else
      {
         //without normal force
         normalTorqueAtCenterOfPressure = torqueAtZeroInPlaneFrame.getZ();
      }

      centerOfPressureToPack.set(centerOfPressurePlaneFrame, vector12x, vector12y);
      return normalTorqueAtCenterOfPressure;
   }

   public double resolveCenterOfPressureAndNormalTorque(FixedFramePoint3DBasics centerOfPressureToPack,
                                                        SpatialForceReadOnly spatialForceVector,
                                                        ReferenceFrame centerOfPressurePlaneFrame)
   {
      // First resolve the wrench at the plane origin:
      wrenchResolvedOnPlane.setIncludingFrame(spatialForceVector);
      wrenchResolvedOnPlane.changeFrame(centerOfPressurePlaneFrame);

      torqueAtZeroInPlaneFrame.set(wrenchResolvedOnPlane.getAngularPart());
      forceInPlaneFrame.set(wrenchResolvedOnPlane.getLinearPart());

      double fz = forceInPlaneFrame.getZ();

      double vector12x = Double.NaN;
      double vector12y = Double.NaN;

      double normalTorqueAtCenterOfPressure;
      if (fz > 1e-7)
      {
         //with sufficient normal force
         vector12x = -1.0 / fz * torqueAtZeroInPlaneFrame.getY();
         vector12y = 1.0 / fz * torqueAtZeroInPlaneFrame.getX();
         normalTorqueAtCenterOfPressure = torqueAtZeroInPlaneFrame.getZ() - vector12x * forceInPlaneFrame.getY() + vector12y * forceInPlaneFrame.getX();
      }
      else
      {
         //without normal force
         normalTorqueAtCenterOfPressure = torqueAtZeroInPlaneFrame.getZ();
      }

      // TODO Implement FixedFramePoint3DBasics.setMatchingFrame(ReferenceFrame, double, double, double)
      centerOfPressureToPack.set(vector12x, vector12y, 0.0);
      centerOfPressurePlaneFrame.transformFromThisToDesiredFrame(centerOfPressureToPack.getReferenceFrame(), centerOfPressureToPack);
      return normalTorqueAtCenterOfPressure;
   }
}
