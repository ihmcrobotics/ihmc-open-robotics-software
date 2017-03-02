package us.ihmc.simulationconstructionset.util;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointsHolder;

public class LinearStickSlipGroundContactModel implements GroundContactModel
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final long serialVersionUID = -2481515446904072547L;

   private static final double
      DEFAULT_K_XY = 1422, DEFAULT_B_XY = 15.6, DEFAULT_K_Z = 125, DEFAULT_B_Z = 300;
   private static final double DEFAULT_STIFFENING_LENGTH = 0.008;
   private static final double DEFAULT_ALPHA_SLIP = 0.7;
   private static final double DEFAULT_ALPHA_STICK = 0.7;

   private final DoubleYoVariable groundKxy = new DoubleYoVariable("groundKxy", "LinearStickSlipGroundContactModel x and y spring constant", registry);
   private final DoubleYoVariable groundBxy = new DoubleYoVariable("groundBxy", "LinearStickSlipGroundContactModel x and y damping constant", registry);
   private final DoubleYoVariable groundKz = new DoubleYoVariable("groundKz", "LinearStickSlipGroundContactModel z spring constant", registry);
   private final DoubleYoVariable groundBz = new DoubleYoVariable("groundBz", "LinearStickSlipGroundContactModel z damping constant", registry);
   private final DoubleYoVariable groundStiffeningLength = new DoubleYoVariable("groundStiffeningLength",
                                                              "LinearStickSlipGroundContactModel z spring nominal stiffening length", registry);
   private final DoubleYoVariable groundAlphaSlip = new DoubleYoVariable("groundAlphaSlip", "LinearStickSlipGroundContactModel slip coefficient of friction",
                                                       registry);
   private final DoubleYoVariable groundAlphaStick = new DoubleYoVariable("groundAlphaStick",
                                                        "LinearStickSlipGroundContactModel stick coefficient of friction", registry);

   private final BooleanYoVariable groundEnableSlip = new BooleanYoVariable("groundEnableSlip", "LinearStickSlipGroundContactModel. If true can slip",
                                                         registry);

   
   private final BooleanYoVariable groundEnableSurfaceNormal = new BooleanYoVariable("groundEnableSurfaceNormal", "LinearStickSlipGroundContactModel. If true will take into account surface normals in computations.",
         registry);

   private ArrayList<GroundContactPoint> groundContactPoints;
   private GroundProfile3D groundProfile3D;

   public LinearStickSlipGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, YoVariableRegistry parentRegistry)
   {
      this(groundContactPointsHolder, DEFAULT_K_XY, DEFAULT_B_XY, DEFAULT_K_Z, DEFAULT_B_Z, DEFAULT_ALPHA_SLIP, DEFAULT_ALPHA_STICK, parentRegistry);
   }

   public LinearStickSlipGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, double alphaSlip, double alphaStick, YoVariableRegistry parentRegistry)
   {
      this(groundContactPointsHolder, DEFAULT_K_XY, DEFAULT_B_XY, DEFAULT_K_Z, DEFAULT_B_Z, alphaSlip, alphaStick, parentRegistry);
   }

   public LinearStickSlipGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, int groundContactGroupIdentifier, double groundKxy, double groundBxy, double groundKz,
           double groundBz, YoVariableRegistry parentRegistry)
   {
      this(groundContactPointsHolder, groundContactGroupIdentifier, groundKxy, groundBxy, groundKz, groundBz, DEFAULT_ALPHA_SLIP, DEFAULT_ALPHA_STICK, parentRegistry);
   }

   public LinearStickSlipGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, double groundKxy, double groundBxy, double groundKz, double groundBz, double groundAlphaSlip,
           double groundAlphaStick, YoVariableRegistry parentRegistry)
   {
      this(groundContactPointsHolder, 0, groundKxy, groundBxy, groundKz, groundBz, groundAlphaSlip, groundAlphaStick, parentRegistry);
   }

   public LinearStickSlipGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, int groundContactGroupIdentifier, double groundKxy, double groundBxy, double groundKz,
           double groundBz, double groundAlphaSlip, double groundAlphaStick, YoVariableRegistry parentRegistry)
   {
      this.groundContactPoints = groundContactPointsHolder.getGroundContactPoints(groundContactGroupIdentifier);

      this.groundKxy.set(groundKxy);
      this.groundBxy.set(groundBxy);
      this.groundKz.set(groundKz);
      this.groundBz.set(groundBz);
      this.groundAlphaSlip.set(groundAlphaSlip);
      this.groundAlphaStick.set(groundAlphaStick);
      this.groundStiffeningLength.set(DEFAULT_STIFFENING_LENGTH);

      groundEnableSlip.set(true);
      groundEnableSurfaceNormal.set(true);
      
      parentRegistry.addChild(registry);
   }

   public void enableSlipping()
   {
      this.groundEnableSlip.set(true);
   }

   public void disableSlipping()
   {
      this.groundEnableSlip.set(false);
   }
   
   public void enableSurfaceNormal()
   {
      this.groundEnableSurfaceNormal.set(true);
   }

   public void disableSurfaceNormal()
   {
      this.groundEnableSurfaceNormal.set(false);
   }

   public void setGroundStiffeningLength(double groundStiffeningLength)
   {
      this.groundStiffeningLength.set(groundStiffeningLength);
   }

   public void setAlphaStickSlip(double groundAlphaStick, double groundAlphaSlip)
   {
      this.groundAlphaStick.set(groundAlphaStick);
      this.groundAlphaSlip.set(groundAlphaSlip);
   }

   public void setXYStiffness(double xyStiffness)
   {
      this.groundKxy.set(xyStiffness);
   }

   public void setZStiffness(double zStiffness)
   {
      this.groundKz.set(zStiffness);
   }

   public void setXYDamping(double xyDamping)
   {
      this.groundBxy.set(xyDamping);
   }

   public void setZDamping(double zDamping)
   {
      this.groundBz.set(zDamping);
   }

   @Override
   public void setGroundProfile3D(GroundProfile3D profile3D)
   {
      this.groundProfile3D = profile3D;
   }

   @Override
   public GroundProfile3D getGroundProfile3D()
   {
      return groundProfile3D;
   }

   @Override
   public void doGroundContact()
   {
      if (groundAlphaStick.getDoubleValue() < groundAlphaSlip.getDoubleValue())
         throw new RuntimeException("alpha stick < alpha slip!");

      for (int i = 0; i < groundContactPoints.size(); i++)
      {
         GroundContactPoint groundContactPoint = groundContactPoints.get(i);
         doGroundContact(groundContactPoint);
      }

      zeroOutTemporaryVariables();
   }

   private final Point3D intersectionPositionInWorld = new Point3D();
   private final Vector3D surfaceNormalTemp = new Vector3D();

   private boolean checkIfInContactUsingProfile3D(GroundContactPoint groundContactPoint)
   {
      boolean isInside;

      if (groundProfile3D == null)
      {
         isInside = (groundContactPoint.getZ() < 0.0);
         intersectionPositionInWorld.set(groundContactPoint.getX(), groundContactPoint.getY(), 0.0);
         surfaceNormalTemp.set(0.0, 0.0, 1.0);
      }
      else if (!groundProfile3D.isClose(groundContactPoint.getX(), groundContactPoint.getY(), groundContactPoint.getZ()))
      {
         isInside = false;
      }
      else
      {
         // TODO: We'll be using the surface normal at the current point, rather than at the touchdown point. We might want to store the touchdown normal and use it instead...
         isInside = groundProfile3D.checkIfInside(groundContactPoint.getX(), groundContactPoint.getY(), groundContactPoint.getZ(), intersectionPositionInWorld,
        		 surfaceNormalTemp);
      }

      if (isInside)
      {
         if (!groundContactPoint.isInContact())
         {
            groundContactPoint.setInContact();
            groundContactPoint.setTouchdownToCurrentLocation();
            groundContactPoint.setSurfaceNormal(surfaceNormalTemp);
         }
      }

      return isInside;
   }

   private void doGroundContact(GroundContactPoint groundContactPoint)
   {
      // If the point is disabled, then no forces:
      if (groundContactPoint.isDisabled())
      {
         groundContactPoint.setForce(0.0, 0.0, 0.0);

         return;
      }

      boolean inContact = checkIfInContactUsingProfile3D(groundContactPoint);

      if (!inContact)
      {
         groundContactPoint.setNotInContact();
         groundContactPoint.setIsSlipping(false);
         groundContactPoint.setForce(0.0, 0.0, 0.0);

         return;
      }

      // If the foot hit, then apply a reaction force:
      resolveContactForce(groundContactPoint);
      checkIfSlipping(groundContactPoint);
   }

   private final Point3D touchdownLocation = new Point3D();
   private final Point3D position = new Point3D();
   private final Vector3D deltaPositionFromTouchdown = new Vector3D();
   private final Vector3D velocity = new Vector3D();

   private void resolveContactForce(GroundContactPoint groundContactPoint)
   {
      groundContactPoint.getTouchdownLocation(touchdownLocation);
      groundContactPoint.getPosition(position);
      groundContactPoint.getVelocity(velocity);
      
      deltaPositionFromTouchdown.sub(touchdownLocation, position);
      
      if (groundEnableSurfaceNormal.getBooleanValue())
      {
         resolveContactForceUsingSurfaceNormal(deltaPositionFromTouchdown, velocity, groundContactPoint);
      }
      else
      {
         resolveContactForceZUp(deltaPositionFromTouchdown, velocity, groundContactPoint);
      }
   }
   
   
   private final Vector3D inPlaneVector1 = new Vector3D();
   private final Vector3D inPlaneVector2 = new Vector3D();
   private void resolveContactForceUsingSurfaceNormal(Vector3D deltaPositionFromTouchdown, Vector3D velocity, GroundContactPoint groundContactPoint)
   {
	   groundContactPoint.getSurfaceNormal(surfaceNormalTemp);
	   
      tempVector.set(0.0, 1.0, 0.0);

      if ((tempVector.dot(surfaceNormalTemp) == 1.0) || (tempVector.dot(surfaceNormalTemp) == -1.0))    // check if they are parallel, in which case UNIT_X will do.
      {
         tempVector.set(1.0, 0.0, 0.0);
      }

      inPlaneVector1.cross(tempVector, surfaceNormalTemp);
      inPlaneVector1.normalize();

      inPlaneVector2.cross(surfaceNormalTemp, inPlaneVector1);
      inPlaneVector2.normalize();
      
      // Spring part
      double xPrime = inPlaneVector1.dot(deltaPositionFromTouchdown);
      double yPrime = inPlaneVector2.dot(deltaPositionFromTouchdown);
      double zPrime = surfaceNormalTemp.dot(deltaPositionFromTouchdown);
      
      forceParallel.set(inPlaneVector1);
      forceParallel.scale(xPrime);
      forceParallel.scaleAdd(yPrime, inPlaneVector2, forceParallel);
      forceParallel.scale(groundKxy.getDoubleValue());
      forceNormal.set(surfaceNormalTemp);
      
      if (groundStiffeningLength.getDoubleValue() - zPrime > 0.002)
      {
         forceNormal.scale(groundKz.getDoubleValue() * zPrime / (groundStiffeningLength.getDoubleValue() - zPrime));
      }
      else
      {
         forceNormal.scale(groundKz.getDoubleValue() * zPrime / 0.002);
      }
      
      // Damping part
      xPrime = inPlaneVector1.dot(velocity);
      yPrime = inPlaneVector2.dot(velocity);
      zPrime = surfaceNormalTemp.dot(velocity);
      forceParallel.scaleAdd(-groundBxy.getDoubleValue() * xPrime, inPlaneVector1, forceParallel);
      forceParallel.scaleAdd(-groundBxy.getDoubleValue() * yPrime, inPlaneVector2, forceParallel);
      forceNormal.scaleAdd(-groundBz.getDoubleValue() * zPrime, surfaceNormalTemp, forceNormal);
      
      double magnitudeOfForceNormal = forceNormal.dot(surfaceNormalTemp);
      if (magnitudeOfForceNormal < 0.0)
      {
         // If both the ground is pulling the point in rather than pushing it out, 
         // and the point is higher than the touchdown point, then set not in contact so that
         // the touchdown point can be reset next tick if still below the ground...
         if (zPrime < 0.0)
         {
            forceParallel.set(0.0, 0.0, 0.0);
            forceNormal.set(0.0, 0.0, 0.0);
            groundContactPoint.setNotInContact();
         }
         else
         {
//            forceParallel.set(0.0, 0.0, 0.0);
            forceNormal.set(0.0, 0.0, 0.0);
         }
      }

      // Sum the total
      forceWorld.set(forceParallel);
      forceWorld.add(forceNormal);
      groundContactPoint.setForce(forceWorld);
   }

   private void resolveContactForceZUp(Vector3D deltaPositionFromTouchdown, Vector3D velocity, GroundContactPoint groundContactPoint)
   {
      // TODO: Use actual surface normal for these forces. See ExperimentalLinearStickSlipGroundContactModel for an implementation.
      double xForce = groundKxy.getDoubleValue() * (deltaPositionFromTouchdown.getX()) - groundBxy.getDoubleValue() * velocity.getX();
      double yForce = groundKxy.getDoubleValue() * (deltaPositionFromTouchdown.getY()) - groundBxy.getDoubleValue() * velocity.getY();

      double zForce;
      if (groundStiffeningLength.getDoubleValue() -deltaPositionFromTouchdown.getZ() > 0.002)
      {
         zForce =
            groundKz.getDoubleValue() * deltaPositionFromTouchdown.getZ() / (groundStiffeningLength.getDoubleValue() -deltaPositionFromTouchdown.getZ())
            - groundBz.getDoubleValue() * velocity.getZ();
      }
      else
      {
         zForce = groundKz.getDoubleValue() * deltaPositionFromTouchdown.getZ() / (0.002) - groundBz.getDoubleValue() * velocity.getZ();
      }

      if (zForce < 0.0)
      {
         // If both the ground is pulling the point in rather than pushing it out, 
         // and the point is higher than the touchdown point, then set not in contact so that
         // the touchdown point can be reset next tick if still below the ground...
         if (deltaPositionFromTouchdown.getZ() < 0.0)
         {
            xForce = 0.0;
            yForce = 0.0;
            zForce = 0.0;
            groundContactPoint.setNotInContact();
         }
         else
         {
//            xForce = 0.0;
//            yForce = 0.0;
            zForce = 0.0;
         }
      }

      groundContactPoint.setForce(xForce, yForce, zForce);
   }

   private final Point3D touchDownPoint = new Point3D();
   private final Vector3D tempVector = new Vector3D();
   private final Vector3D
      forceWorld = new Vector3D(), forceNormal = new Vector3D(), forceParallel = new Vector3D();

   private void checkIfSlipping(GroundContactPoint groundContactPoint)
   {
      if (!groundEnableSlip.getBooleanValue())
      {
         groundContactPoint.setIsSlipping(false);

         return;
      }

      // Stick-Slip code below
      // Compute the horizontal to vertical force ratio:
      groundContactPoint.getForce(forceWorld);
      groundContactPoint.getSurfaceNormal(surfaceNormalTemp);
      
      forceNormal.set(surfaceNormalTemp);
      forceNormal.scale(surfaceNormalTemp.dot(forceWorld));

      forceParallel.set(forceWorld);
      forceParallel.sub(forceNormal);

      double parallelSpringForce = forceParallel.length();
      double normalSpringForce = forceNormal.length();

      double ratio = parallelSpringForce / normalSpringForce;

      // It's slipping if it already was and forces are above dynamic ratio.  It starts slipping if above static ratio.
      // But don't slip if inside the ground by more than 1 cm since this probably only occurs when in a wedge and keep sliding
      // perpendicular to the normal into the chasm..
      // +++JEP: 140626: Revisit the chasm thing later. For now take the heightAt check out...
//    if ((gc.getZ() > heightAt - 0.010) && ((ratio > groundAlphaStick.getDoubleValue()) || ((gc.isSlipping()) && (ratio > groundAlphaSlip.getDoubleValue()))))
      if ((ratio > groundAlphaStick.getDoubleValue()) || (groundContactPoint.isSlipping()) && (ratio > groundAlphaSlip.getDoubleValue()))
      {
         groundContactPoint.setIsSlipping(true);
         double parallelSlipForce = groundAlphaSlip.getDoubleValue() * normalSpringForce;

         double parallelScale = parallelSlipForce / parallelSpringForce;
         if (parallelScale < 1.0)
            forceParallel.scale(parallelScale);

         forceWorld.add(forceNormal, forceParallel);
         groundContactPoint.setForce(forceWorld);

         // Move touch-down values along the perp direction to follow the slipping.

         double len = forceParallel.length();
         if (len > 1e-7)
            forceParallel.scale(1.0 / len);

         groundContactPoint.getPosition(tempVector);
         groundContactPoint.getTouchdownLocation(touchDownPoint);
         tempVector.sub(touchDownPoint);

         forceParallel.scale(-0.05 * tempVector.length());

         touchDownPoint.add(forceParallel);
         groundContactPoint.setTouchdownLocation(touchDownPoint);
         
         // The following is a little inefficient since we call checkIfInside twice for any point that is slipping, but 
         // cleaner than letting the next tick know we need to update the surface normal due to a slip...
         boolean isInside;
         if (groundProfile3D == null)
         {
            isInside = (groundContactPoint.getZ() < 0.0);
            intersectionPositionInWorld.set(groundContactPoint.getX(), groundContactPoint.getY(), 0.0);
            surfaceNormalTemp.set(0.0, 0.0, 1.0);
         }
         else
         {
            // TODO: We'll be using the surface normal at the current point, rather than at the touchdown point. We might want to store the touchdown normal and use it instead...
            isInside = groundProfile3D.checkIfInside(groundContactPoint.getX(), groundContactPoint.getY(), groundContactPoint.getZ(), intersectionPositionInWorld,
                surfaceNormalTemp);
         }
         
         if (isInside) groundContactPoint.setSurfaceNormal(surfaceNormalTemp);
      }
      else
      {
         groundContactPoint.setIsSlipping(false);
      }
   }

   private void zeroOutTemporaryVariables()
   {
      // Zero these temporary variables out so that rewindability tests which use reflection don't pick them up as changed state variables.
      intersectionPositionInWorld.set(0.0, 0.0, 0.0);
      surfaceNormalTemp.set(0.0, 0.0, 0.0);

      touchDownPoint.set(0.0, 0.0, 0.0);
      tempVector.set(0.0, 0.0, 0.0);
      forceWorld.set(0.0, 0.0, 0.0);
      forceNormal.set(0.0, 0.0, 0.0);
      forceParallel.set(0.0, 0.0, 0.0);
   }

}
