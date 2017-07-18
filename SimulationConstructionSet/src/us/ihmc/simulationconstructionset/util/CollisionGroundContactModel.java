package us.ihmc.simulationconstructionset.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.MovingGroundContactModel;
import us.ihmc.simulationconstructionset.MovingGroundProfile;
import us.ihmc.simulationconstructionset.Robot;

public class CollisionGroundContactModel implements GroundContactModel, MovingGroundContactModel
{
   private static final long serialVersionUID = -1038863972028441303L;

   private static final double defaultCoefficientOfRestitution = 0.5;
   private static final double defaultCoefficientOfFriction = 0.7;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("CollisionGroundContactModel");
   private YoDouble groundRestitution, groundFriction;
   
   private ArrayList<GroundContactPoint> gcPoints;
   private GroundProfile3D profile3D;
   private MovingGroundProfile movingProfile;

   private final Vector3D tempForceOne = new Vector3D();
   private final Vector3D tempForceTwo = new Vector3D();
   // private boolean movingGround = false;
   private boolean microCollision;

   private Map<GroundContactPoint, YoBoolean> isInsideSpies = new HashMap<>();
   private Map<GroundContactPoint, YoBoolean> microCollisionSpies = new HashMap<>();
   private Map<GroundContactPoint, YoDouble> closestIntersectionSpies = new HashMap<>();
   private Map<GroundContactPoint, YoBoolean> contactIsCloseToGround = new HashMap<>();

   public CollisionGroundContactModel(Robot rob, YoVariableRegistry parentRegistry)
   {
      this(rob, defaultCoefficientOfRestitution, defaultCoefficientOfFriction, parentRegistry);
   }
   
   public CollisionGroundContactModel(Robot rob, double epsilon, double mu, YoVariableRegistry parentRegistry)
   {
      this(rob, 0, epsilon, mu, parentRegistry);
   }
   
   
   public CollisionGroundContactModel(Robot robot, int groundContactGroupIdentifier, double epsilon, double mu, YoVariableRegistry parentRegistry)
   {
      this(robot.getGroundContactPoints(groundContactGroupIdentifier), epsilon, mu, parentRegistry);
   }

   public CollisionGroundContactModel(ArrayList<GroundContactPoint> gcPoints, YoVariableRegistry parentRegistry)
   {
      this(gcPoints, defaultCoefficientOfRestitution, defaultCoefficientOfFriction, parentRegistry);
   }

   public CollisionGroundContactModel(ArrayList<GroundContactPoint> gcPoints, double epsilon, double mu, YoVariableRegistry parentRegistry)
   {
      this.gcPoints = gcPoints;

      for(GroundContactPoint groundContactPoint : gcPoints)
      {
         isInsideSpies.put(groundContactPoint, new YoBoolean(groundContactPoint.getName() + "_isInsideSpy", registry));
         microCollisionSpies.put(groundContactPoint, new YoBoolean(groundContactPoint.getName() + "_microCollisionSpy", registry));
         closestIntersectionSpies.put(groundContactPoint, new YoDouble(groundContactPoint.getName() + "_closestIntersectionZ", registry));
         contactIsCloseToGround.put(groundContactPoint, new YoBoolean(groundContactPoint.getName() + "_isCloseToGround", registry));
      }

      groundRestitution = new YoDouble("groundRestitution", "CollisionGroundContactModel coefficient Of Restitution", registry);
      groundFriction = new YoDouble("groundFriction", "CollisionGroundContactModel coefficient Of Friction", registry);

      addRegistryToParent(parentRegistry);

      groundRestitution.set(epsilon);
      groundFriction.set(mu);

      this.initGroundContact();
   }
   
   @Override
   public void setGroundProfile3D(GroundProfile3D profile3D)
   {
      this.profile3D = profile3D;
      this.movingProfile = null;
   }
   
   @Override
   public GroundProfile3D getGroundProfile3D()
   {
      return profile3D;
   }

   @Override
   public void setGroundProfile(MovingGroundProfile profile)
   {
      this.profile3D = profile;
      this.movingProfile = null;
   }

   private void initGroundContact()
   {
   }

   private Vector3D normalVector = new Vector3D(0.0, 0.0, 1.0);
   private Vector3D velocityVector = new Vector3D(0.0, 0.0, 0.0);
   private Vector3D p_world = new Vector3D();

   boolean iterateForward = true;

   int jj = 0;

/*   public void doGroundContact2()
   {
     jj++;
     if (jj == gcPoints.size()) jj = 0;


         GroundContactPoint gc = (GroundContactPoint) gcPoints.get(jj);
         doGroundContact(gc);



   }
*/


   @Override
   public void doGroundContact()
   {
      iterateForward = !iterateForward;

      if (iterateForward)
      {
         for (int i = 0; i < gcPoints.size(); i++)
         {
            GroundContactPoint gc = gcPoints.get(i);
            doGroundContact(gc);
         }
      }

      else
      {
         for (int i = gcPoints.size() - 1; i >= 0; i--)
         {
            GroundContactPoint gc = gcPoints.get(i);
            doGroundContact(gc);
         }
      }


   }

   private final Point3D closestIntersection = new Point3D();

   private void doGroundContact(GroundContactPoint gc)
   {
      // First, do the callback if a point has it:
      // ExternalForcePointUpdater externalForcePointUpdater = gc.getExternalForcePointUpdater();
      // if (externalForcePointUpdater != null) externalForcePointUpdater.updateExternalForcePoint(gc);

      // If the point is turned off (fs is set to -1 or less), then no forces:
      if (gc.isDisabled())
      {
         gc.setForce(0.0, 0.0, 0.0);

         return;
      }

      // See if point hit the ground or not:
      boolean isInside = false;
      if (profile3D != null)
      {
         isInside = profile3D.checkIfInside(gc.getX(), gc.getY(), gc.getZ(), closestIntersection, normalVector);
         closestIntersectionSpies.get(gc).set(closestIntersection.getZ());
      }
      isInsideSpies.get(gc).set(isInside);

      if (isInside)
      {
         if (!gc.isInContact())
         {
            microCollision = false;
            gc.setInContact();
            gc.setTouchdownToCurrentLocation();

            // System.out.println(gc + " hit the ground");
         }

         else
         {
            microCollision = true;
         }
         microCollisionSpies.get(gc).set(microCollision);
      }

      else
         gc.setNotInContact();

      contactIsCloseToGround.get(gc).set((profile3D != null) && (profile3D.isClose(gc.getX(), gc.getY(), gc.getZ())));
      if (!contactIsCloseToGround.get(gc).getBooleanValue())
         return;

      // If the foot hit, then y an impulse:

      if (gc.isInContact())
      {
//         if (profile3D != null)
//         {
//            profile3D.closestIntersectionAndNormalAt(gc.getX(), gc.getY(), gc.getZ(), closestIntersection, normalVector);
//         }
//         else
//            normalVector.set(0.0, 0.0, 1.0);

         if (movingProfile != null)
         {
            movingProfile.velocityAt(gc.getX(), gc.getY(), gc.getZ(), velocityVector);
         }
         else
            velocityVector.set(0.0, 0.0, 0.0);

         if (microCollision)
         {
            double penetration_squared;
            if (profile3D != null)
            {
               penetration_squared = (gc.getX() - closestIntersection.getX()) * (gc.getX() - closestIntersection.getX())
                                     + (gc.getY() - closestIntersection.getY()) * (gc.getY() - closestIntersection.getY())
                                     + (gc.getZ() - closestIntersection.getZ()) * (gc.getZ() - closestIntersection.getZ());
            }
            else
               penetration_squared = gc.getZ() * gc.getZ();

            gc.resolveMicroCollision(penetration_squared, velocityVector, normalVector, groundRestitution.getDoubleValue(), groundFriction.getDoubleValue(), p_world);

            // gc.resolveMicroCollision(velocityVector, normalVector, 3.5, mu, p_world);
            // gc.resolveCollision(velocityVector, normalVector, epsilon, mu, p_world);

            double RATE = 0.0;
            
            gc.getForce(tempForceOne);
            tempForceTwo.set(p_world);
            tempForceTwo.scale(RATE);
            tempForceOne.add(tempForceTwo);
            gc.setForce(tempForceOne);

         }
         else
         {
            if(gc.getParentJoint() != null)
               gc.resolveCollision(velocityVector, normalVector, groundRestitution.getDoubleValue(), groundFriction.getDoubleValue(), p_world);
         }


         gc.incrementCollisionCount();

         // Put the impulse in f so I can see it:

         /*
          * gc.fx.set(p_world.x);
          * gc.fy.set(p_world.y);
          * gc.fz.set(p_world.z);
          */

         if (p_world.getZ() < 0.0)
         {
            // System.out.println("Negative impulse!!!");
            // System.out.println("p_world = " + p_world);
            // System.exit(0);
         }



         /*
          * gc.fx.set(K_XY * (gc.tdx.getDoubleValue()- gc.getX()) - B_XY * gc.getXVelocity());
          * gc.fy.set(K_XY * (gc.tdy.getDoubleValue()- gc.getY()) - B_XY * gc.getYVelocity());
          *
          * if (NOMLEN + (gc.getZ() - gc.tdz.getDoubleValue()) > 0.002)
          * {
          * gc.fz.set(-K_Z * (gc.getZ() - gc.tdz.getDoubleValue())/(NOMLEN + (gc.getZ() - gc.tdz.getDoubleValue())) - B_Z * gc.getZVelocity());
          * }
          * else
          * {
          * gc.fz.set(-K_Z * (gc.getZ() - gc.tdz.getDoubleValue())/(0.002) - B_Z * gc.getZVelocity());
          * }
          *
          * if (gc.fz.getDoubleValue() < 0.0) gc.fz.set(0.0);
          */
      }
      else
      {
         gc.setForce(0.0, 0.0, 0.0);

         gc.setImpulse(0.0, 0.0, 0.0);
      }
   }
   
   private void addRegistryToParent(YoVariableRegistry parentRegistry)
   {
      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

}
