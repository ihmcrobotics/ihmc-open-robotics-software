package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;

public class StickSlipContactCalculator
{
   private final Point3D contactAPosition = new Point3D();
   private final Point3D contactBPosition = new Point3D();

   private final Vector3D contactAVelocity = new Vector3D();
   private final Vector3D contactBVelocity = new Vector3D();

   private final Vector3D velocityDifference = new Vector3D();

   private final Point3D contactableClosestPoint = new Point3D();
   private final Vector3D contactableSurfaceNormal = new Vector3D();

   private final Vector3D contactForceDirection = new Vector3D();

   private final Vector3D contactForceFromSpring = new Vector3D();
   private final Vector3D contactForceFromDamper = new Vector3D();
   private final Vector3D totalContactForce = new Vector3D();

   private final Vector3D normalContactForce = new Vector3D();
   private final Vector3D parallelContactForce = new Vector3D();
   private final Vector3D slipVector = new Vector3D();

   public void doContactMade(ExternalForcePoint contactPointA, Contactable contactableB, GroundContactPoint contactPointB)
   {
      contactPointA.getPosition(contactAPosition);
      contactPointB.setOffsetWorld(contactAPosition);
      contactableB.updateContactPoints();
      contactPointB.setIsInContact(true);
      contactPointB.setIsSlipping(false);
   }

   public void doContactBroken(ExternalForcePoint contactPointA, GroundContactPoint contactPointB)
   {
      contactPointB.setIsInContact(false);
      contactPointB.setIsSlipping(false);

      totalContactForce.set(0.0, 0.0, 0.0);

      applyContactForces(contactPointA, contactPointB);
   }

   public void doCurrentlyInContact(ExternalForcePoint contactPointA, Contactable contactableB, GroundContactPoint contactPointB, double kContact,
                                    double bContact, double alphaStick, double alphaSlip)
   {
      computeSpringDamperAndTotalForces(contactPointA, contactableB, contactPointB, kContact, bContact);
      projectTotalForceOntoLineBetweenPoints();
      checkForSlipping(contactableB, contactPointB, alphaStick, alphaSlip);
      applyContactForces(contactPointA, contactPointB);
   }

   private void computeSpringDamperAndTotalForces(ExternalForcePoint contactPointA, Contactable contactableB, GroundContactPoint contactPointB,
         double kContact, double bContact)
   {
      contactPointA.getPosition(contactAPosition);
      contactPointB.getPosition(contactBPosition);
      contactableB.closestIntersectionAndNormalAt(contactableClosestPoint, contactableSurfaceNormal, contactBPosition);

      contactForceDirection.set(contactBPosition);
      contactForceDirection.sub(contactAPosition);

      if (contactForceDirection.length() < 1e-7)
      {
         contactForceFromSpring.set(0.0, 0.0, 0.0);
         contactForceFromDamper.set(0.0, 0.0, 0.0);
      }
      else
      {
         contactForceFromSpring.set(contactForceDirection);
         contactForceFromSpring.scale(kContact);

         contactPointA.getVelocity(contactAVelocity);
         contactPointB.getVelocity(contactBVelocity);

         velocityDifference.set(contactBVelocity);
         velocityDifference.sub(contactAVelocity);

         contactForceFromDamper.set(velocityDifference);
         contactForceFromDamper.scale(bContact);
      }

      totalContactForce.set(contactForceFromSpring);
      totalContactForce.add(contactForceFromDamper);
   }

   private void applyContactForces(ExternalForcePoint contactPointA, GroundContactPoint contactPointB)
   {
      contactPointA.setForce(totalContactForce);
      totalContactForce.negate();
      contactPointB.setForce(totalContactForce);
   }

   private void checkForSlipping(Contactable contactableB, GroundContactPoint contactPointB, double alphaStick, double alphaSlip)
   {
      normalContactForce.set(contactableSurfaceNormal);
      normalContactForce.scale(contactableSurfaceNormal.dot(totalContactForce));

      parallelContactForce.set(totalContactForce);
      parallelContactForce.sub(normalContactForce);

      double parallelForceMagnitude = parallelContactForce.length();
      double normalForceMagnitude = normalContactForce.length();

      double ratio = parallelForceMagnitude / normalForceMagnitude;

      boolean isAlreadySlipping = contactPointB.isSlipping();

      if ((ratio > alphaStick) || (isAlreadySlipping && (ratio > alphaSlip)))
      {
         contactPointB.setIsSlipping(true);

         double parallelSlipForce = alphaSlip * normalForceMagnitude;

         double parallelScale = parallelSlipForce / parallelForceMagnitude;
         if (parallelScale < 1.0)
            parallelContactForce.scale(parallelScale);

         totalContactForce.add(normalContactForce, parallelContactForce);

         // Move touch-down point along the parallel direction to follow the
         // slipping.

         slipVector.set(parallelContactForce);

         if (parallelForceMagnitude > 1e-7)
            slipVector.scale(1.0 / parallelForceMagnitude);

         double distanceBetweenContactPoints = contactAPosition.distance(contactBPosition);
         slipVector.scale(-0.05 * distanceBetweenContactPoints);

         contactBPosition.add(slipVector);
         contactPointB.setOffsetWorld(contactBPosition);
         contactableB.updateContactPoints();
      }
      else
      {
         contactPointB.setIsSlipping(false);
      }
   }

   private void projectTotalForceOntoLineBetweenPoints()
   {
      // Currently this calculates the realistic force and then projects it
      // onto the line between the initial contact point and the current
      // location of the point mass.
      // TODO: Perhaps this should co-locate the external force point for the sphere
      // with the position of the point mass so that damping can be handled
      // more realistically as an arbitrary vector in the direction opposite
      // the relative motion of the two objects?

      contactForceDirection.set(contactBPosition);
      contactForceDirection.sub(contactAPosition);

      if (contactForceDirection.length() > 1e-10)
      {
         contactForceDirection.normalize();
         double dotProduct = totalContactForce.dot(contactForceDirection);
         totalContactForce.set(contactForceDirection);
         totalContactForce.scale(dotProduct);
      }
   }
}
