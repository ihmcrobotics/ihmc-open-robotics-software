package us.ihmc.simulationconstructionset.physics.collision;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.physics.engine.jerry.JointPhysics;

public class CollisionResolutionTest
{
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCollisionResolutionOne() throws UnreasonableAccelerationException
   {
      Robot robot = new Robot("TestCollisions");
      FloatingJoint floatingJoint = new FloatingJoint("base", new Vector3D(), robot);

      Link link = new Link("baseLink");

      double mass = 5e-3;
      double Ixx = 2e-5;
      double Iyy = 2e-5;
      double Izz = 4e-5;
            
      link.setMass(mass);
      link.setMomentOfInertia(Ixx, Iyy, Izz);
      link.setComOffset(new Vector3D());
      floatingJoint.setLink(link);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("externalForcePoint", new Vector3D(), robot);
      floatingJoint.addExternalForcePoint(externalForcePoint);

      robot.addRootJoint(floatingJoint);
      JointPhysics<?> jointPhysics = floatingJoint.physics;

      double downwardVelocity = 1.4;
      Vector3D initialVelocity = new Vector3D(0.0, 0.0, -downwardVelocity);
      
      floatingJoint.setVelocity(initialVelocity);
      floatingJoint.setAngularVelocityInBody(new Vector3D(0.0, 0.0, 0.0));
      robot.doDynamicsButDoNotIntegrate();
      
      RotationMatrix identityMatrix = new RotationMatrix();
      identityMatrix.setIdentity();
      
      // When collision is straight on with the center of mass:
      Vector3D offsetFromCenterOfMass = new Vector3D(0.0, 0.0, 0.0);
      
      Matrix3D kiCollision = jointPhysics.computeKiCollision(offsetFromCenterOfMass, identityMatrix);
      Matrix3D expectedKiCollision = new Matrix3D(1.0/mass, 0.0, 0.0, 0.0, 1.0/mass, 0.0, 0.0, 0.0, 1.0/mass);
      EuclidCoreTestTools.assertMatrix3DEquals("kiCollision = " + kiCollision, expectedKiCollision, kiCollision, 1e-7);
      
      Vector3D velocityOfOtherObjectInWorld = new Vector3D(0.0, 0.0, 0.0);
      Vector3D collisionNormalInWorld = new Vector3D(0.0, 0.0, 1.0);
      double epsilon = 1.0;
      double mu = 0.0;
      Vector3D impulseInWorldToPack = new Vector3D();

      externalForcePoint.resolveCollision(velocityOfOtherObjectInWorld, collisionNormalInWorld, epsilon, mu, impulseInWorldToPack);

      double upwardVelocity = epsilon * downwardVelocity;
      double deltaVelocity = downwardVelocity + upwardVelocity;
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, mass * deltaVelocity), impulseInWorldToPack, 1e-7);
      Vector3D velocity = new Vector3D();
      floatingJoint.getVelocity(velocity);
      
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, upwardVelocity), velocity, 1e-7);
      
      // When collision is offset from the center of mass:
      offsetFromCenterOfMass = new Vector3D(0.1, 0.04, -0.02);
      initialVelocity = new Vector3D(0.1, 0.07, -1.0);

      externalForcePoint.setOffsetJoint(offsetFromCenterOfMass);
      
      floatingJoint.setVelocity(initialVelocity);
      floatingJoint.setAngularVelocityInBody(new Vector3D(0.13, 1.0, 0.11));
      robot.update();
      robot.updateVelocities();
      robot.doDynamicsButDoNotIntegrate();
      
      Vector3D initialVelocityAtImpactPoint = new Vector3D();
      externalForcePoint.getVelocity(initialVelocityAtImpactPoint);
      
      double initialEnergy = computeEnergy(robot);

      kiCollision = jointPhysics.computeKiCollision(offsetFromCenterOfMass, identityMatrix);
      expectedKiCollision = new Matrix3D(1.0/mass, 0.0, 0.0, 0.0, 1.0/mass, 0.0, 0.0, 0.0, 1.0/mass);
      
      Matrix3D rTilde = new Matrix3D(0.0, -offsetFromCenterOfMass.getZ(), offsetFromCenterOfMass.getY(), 
            offsetFromCenterOfMass.getZ(), 0.0, -offsetFromCenterOfMass.getX(),
            -offsetFromCenterOfMass.getY(), offsetFromCenterOfMass.getX(), 0.0);

      Matrix3D IInverse = new Matrix3D(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
      IInverse.invert();
      
      Matrix3D expectedKiCollisionAngularPart = new Matrix3D(rTilde);
      expectedKiCollisionAngularPart.multiply(IInverse);
      expectedKiCollisionAngularPart.multiply(rTilde);
      
      expectedKiCollision.sub(expectedKiCollisionAngularPart);
      
//      System.out.println("expectedKiCollisionAngularPart = " + expectedKiCollisionAngularPart);
//      System.out.println("kiCollision = " + kiCollision);
      EuclidCoreTestTools.assertMatrix3DEquals("kiCollision = " + kiCollision, expectedKiCollision, kiCollision, 1e-7);
      
      velocityOfOtherObjectInWorld = new Vector3D(0.0, 0.0, 0.0);
      collisionNormalInWorld = new Vector3D(0.0, 0.0, 1.0);
      epsilon = 0.7;
      mu = 0.3;
      impulseInWorldToPack = new Vector3D();

      externalForcePoint.resolveCollision(velocityOfOtherObjectInWorld, collisionNormalInWorld, epsilon, mu, impulseInWorldToPack);

      Vector3D deltaV = new Vector3D(impulseInWorldToPack);
      kiCollision.transform(deltaV);
      
      Vector3D expectedFinalVelocity = new Vector3D(initialVelocityAtImpactPoint);
      expectedFinalVelocity.add(deltaV);
      
      Vector3D finalVelocityAtImpactPoint = new Vector3D();
      externalForcePoint.getVelocity(finalVelocityAtImpactPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(expectedFinalVelocity, finalVelocityAtImpactPoint, 1e-7);
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
