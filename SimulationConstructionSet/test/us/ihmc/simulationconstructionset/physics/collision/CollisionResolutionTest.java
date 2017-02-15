package us.ihmc.simulationconstructionset.physics.collision;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.physics.engine.jerry.JointPhysics;
import us.ihmc.tools.testing.JUnitTools;

public class CollisionResolutionTest
{
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCollisionResolutionOne() throws UnreasonableAccelerationException
   {
      Robot robot = new Robot("TestCollisions");
      FloatingJoint floatingJoint = new FloatingJoint("base", new Vector3d(), robot);

      Link link = new Link("baseLink");

      double mass = 5e-3;
      double Ixx = 2e-7;
      double Iyy = 2e-7;
      double Izz = 4e-7;
            
      link.setMass(mass);
      link.setMomentOfInertia(Ixx, Iyy, Izz);
      link.setComOffset(new Vector3d());
      floatingJoint.setLink(link);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("externalForcePoint", new Vector3d(), robot);
      floatingJoint.addExternalForcePoint(externalForcePoint);

      robot.addRootJoint(floatingJoint);
      JointPhysics<?> jointPhysics = floatingJoint.physics;

      double downwardVelocity = 1.4;
      Vector3d initialVelocity = new Vector3d(0.0, 0.0, -downwardVelocity);
      
      floatingJoint.setVelocity(initialVelocity);
      floatingJoint.setAngularVelocityInBody(new Vector3d(0.0, 0.0, 0.0));
      robot.doDynamicsButDoNotIntegrate();
      
      Matrix3d identityMatrix = new Matrix3d();
      identityMatrix.setIdentity();
      
      // When collision is straight on with the center of mass:
      Vector3d offsetFromCenterOfMass = new Vector3d(0.0, 0.0, 0.0);
      
      Matrix3d kiCollision = jointPhysics.computeKiCollision(offsetFromCenterOfMass, identityMatrix);
      Matrix3d expectedKiCollision = new Matrix3d(1.0/mass, 0.0, 0.0, 0.0, 1.0/mass, 0.0, 0.0, 0.0, 1.0/mass);
      JUnitTools.assertMatrix3dEquals("kiCollision = " + kiCollision, expectedKiCollision, kiCollision, 1e-7);
      
      Vector3d velocityOfOtherObjectInWorld = new Vector3d(0.0, 0.0, 0.0);
      Vector3d collisionNormalInWorld = new Vector3d(0.0, 0.0, 1.0);
      double epsilon = 1.0;
      double mu = 0.0;
      Vector3d impulseInWorldToPack = new Vector3d();

      externalForcePoint.resolveCollision(velocityOfOtherObjectInWorld, collisionNormalInWorld, epsilon, mu, impulseInWorldToPack);

      double upwardVelocity = epsilon * downwardVelocity;
      double deltaVelocity = downwardVelocity + upwardVelocity;
      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, mass * deltaVelocity), impulseInWorldToPack, 1e-7);
      Vector3d velocity = new Vector3d();
      floatingJoint.getVelocity(velocity);
      
      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, upwardVelocity), velocity, 1e-7);
      
      // When collision is offset from the center of mass:
      offsetFromCenterOfMass = new Vector3d(0.1, 0.04, -0.02);
      initialVelocity = new Vector3d(0.1, 0.07, -1.0);

      externalForcePoint.setOffsetJoint(offsetFromCenterOfMass);
      
      floatingJoint.setVelocity(initialVelocity);
      floatingJoint.setAngularVelocityInBody(new Vector3d(0.13, 1.0, 0.11));
      robot.update();
      robot.updateVelocities();
      robot.doDynamicsButDoNotIntegrate();
      
      Vector3d initialVelocityAtImpactPoint = new Vector3d();
      externalForcePoint.getVelocity(initialVelocityAtImpactPoint);
      
      double initialEnergy = computeEnergy(robot);

      kiCollision = jointPhysics.computeKiCollision(offsetFromCenterOfMass, identityMatrix);
      expectedKiCollision = new Matrix3d(1.0/mass, 0.0, 0.0, 0.0, 1.0/mass, 0.0, 0.0, 0.0, 1.0/mass);
      
      Matrix3d rTilde = new Matrix3d(0.0, -offsetFromCenterOfMass.getZ(), offsetFromCenterOfMass.getY(), 
            offsetFromCenterOfMass.getZ(), 0.0, -offsetFromCenterOfMass.getX(),
            -offsetFromCenterOfMass.getY(), offsetFromCenterOfMass.getX(), 0.0);

      Matrix3d IInverse = new Matrix3d(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
      IInverse.invert();
      
      Matrix3d expectedKiCollisionAngularPart = new Matrix3d(rTilde);
      expectedKiCollisionAngularPart.mul(IInverse);
      expectedKiCollisionAngularPart.mul(rTilde);
      
      expectedKiCollision.sub(expectedKiCollisionAngularPart);
      
//      System.out.println("expectedKiCollisionAngularPart = " + expectedKiCollisionAngularPart);
//      System.out.println("kiCollision = " + kiCollision);
      JUnitTools.assertMatrix3dEquals("kiCollision = " + kiCollision, expectedKiCollision, kiCollision, 1e-7);
      
      velocityOfOtherObjectInWorld = new Vector3d(0.0, 0.0, 0.0);
      collisionNormalInWorld = new Vector3d(0.0, 0.0, 1.0);
      epsilon = 0.7;
      mu = 0.3;
      impulseInWorldToPack = new Vector3d();

      externalForcePoint.resolveCollision(velocityOfOtherObjectInWorld, collisionNormalInWorld, epsilon, mu, impulseInWorldToPack);

      Vector3d deltaV = new Vector3d(impulseInWorldToPack);
      kiCollision.transform(deltaV);
      
      Vector3d expectedFinalVelocity = new Vector3d(initialVelocityAtImpactPoint);
      expectedFinalVelocity.add(deltaV);
      
      Vector3d finalVelocityAtImpactPoint = new Vector3d();
      externalForcePoint.getVelocity(finalVelocityAtImpactPoint);
      
      JUnitTools.assertTuple3dEquals(expectedFinalVelocity, finalVelocityAtImpactPoint, 1e-7);
      double finalEnergy = computeEnergy(robot);
      
//      System.out.println("initialEnergy = " + initialEnergy);
//      System.out.println("finalEnergy = " + finalEnergy);
      assertTrue("initialEnergy = " + initialEnergy + ", finalEnergy = " + finalEnergy, finalEnergy <= initialEnergy);
   }

   private double computeEnergy(Robot robot)
   {
      double rotationalEnergy = robot.computeRotationalKineticEnergy();
      double translationalEnergy = robot.computeTranslationalKineticEnergy();
      double totalEnergy = rotationalEnergy + translationalEnergy;
      
      return totalEnergy;
   }

}
