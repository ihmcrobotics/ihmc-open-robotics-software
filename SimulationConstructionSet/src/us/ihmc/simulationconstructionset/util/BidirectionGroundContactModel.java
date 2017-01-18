package us.ihmc.simulationconstructionset.util;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointsHolder;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class BidirectionGroundContactModel implements GroundContactModel
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final long serialVersionUID = -2481515446904072547L;

   private static final double DEFAULT_K_XY = 1422, DEFAULT_B_XY = 15.6, DEFAULT_K_Z = 125, DEFAULT_B_Z = 300;

   private final DoubleYoVariable groundKxy = new DoubleYoVariable("groundKxy", "BidirectionGroundContactModel x and y spring constant", registry);
   private final DoubleYoVariable groundBxy = new DoubleYoVariable("groundBxy", "BidirectionalGroundContactModel x and y damping constant", registry);
   private final DoubleYoVariable groundKz = new DoubleYoVariable("groundKz", "BidirectionalGroundContactModel z spring constant", registry);
   private final DoubleYoVariable groundBz = new DoubleYoVariable("groundBz", "BidirectionalGroundContactModel z damping constant", registry);

   private ArrayList<GroundContactPoint> groundContactPoints;
   private GroundProfile3D groundProfile3D;

   public BidirectionGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, YoVariableRegistry parentRegistry)
   {
      this(groundContactPointsHolder, DEFAULT_K_XY, DEFAULT_B_XY, DEFAULT_K_Z, DEFAULT_B_Z, parentRegistry);
   }

   public BidirectionGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, double groundKxy, double groundBxy, double groundKz,
         double groundBz, YoVariableRegistry parentRegistry)
   {
      this(groundContactPointsHolder, 0, groundKxy, groundBxy, groundKz, groundBz, parentRegistry);
   }

   public BidirectionGroundContactModel(GroundContactPointsHolder groundContactPointsHolder, int groundContactGroupIdentifier, double groundKxy,
         double groundBxy, double groundKz, double groundBz, YoVariableRegistry parentRegistry)
   {
      this.groundContactPoints = groundContactPointsHolder.getGroundContactPoints(groundContactGroupIdentifier);

      this.groundKxy.set(groundKxy);
      this.groundBxy.set(groundBxy);
      this.groundKz.set(groundKz);
      this.groundBz.set(groundBz);

      parentRegistry.addChild(registry);
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

   public void setGroundProfile3D(GroundProfile3D profile3D)
   {
      this.groundProfile3D = profile3D;
   }

   public GroundProfile3D getGroundProfile3D()
   {
      return groundProfile3D;
   }

   public void doGroundContact()
   {
      for (int i = 0; i < groundContactPoints.size(); i++)
      {
         doGroundContact(groundContactPoints.get(i));
      }
   }

   private void doGroundContact(GroundContactPoint groundContactPoint)
   {
      if (groundContactPoint.isDisabled())
      {
         groundContactPoint.setForce(0.0, 0.0, 0.0);
         return;
      }

      boolean inContact = groundContactPoint.isInContact();

      if (!inContact)
      {
         groundContactPoint.setNotInContact();
         groundContactPoint.setIsSlipping(false);
         groundContactPoint.setForce(0.0, 0.0, 0.0);

         return;
      }

      resolveContactForce(groundContactPoint);
   }

   private final Point3d touchdownLocation = new Point3d();
   private final Point3d position = new Point3d();
   private final Vector3d deltaPositionFromTouchdown = new Vector3d();
   private final Vector3d velocity = new Vector3d();

   private void resolveContactForce(GroundContactPoint groundContactPoint)
   {
      groundContactPoint.getTouchdownLocation(touchdownLocation);
      groundContactPoint.getPosition(position);
      groundContactPoint.getVelocity(velocity);

      deltaPositionFromTouchdown.sub(touchdownLocation, position);

      resolveContactForceZUp(deltaPositionFromTouchdown, velocity, groundContactPoint);
   }

   private void resolveContactForceZUp(Vector3d deltaPositionFromTouchdown, Vector3d velocity, GroundContactPoint groundContactPoint)
   {
      double xForce = groundKxy.getDoubleValue() * (deltaPositionFromTouchdown.getX() - groundBxy.getDoubleValue()) * velocity.getX();
      double yForce = groundKxy.getDoubleValue() * (deltaPositionFromTouchdown.getY() - groundBxy.getDoubleValue()) * velocity.getY();

      double zForce = groundKz.getDoubleValue() * deltaPositionFromTouchdown.getZ() / (0.002) - groundBz.getDoubleValue() * velocity.getZ();

      groundContactPoint.setForce(xForce, yForce, zForce);
   }
}
