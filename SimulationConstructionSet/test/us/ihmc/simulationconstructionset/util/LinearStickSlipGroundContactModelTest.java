package us.ihmc.simulationconstructionset.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointsHolder;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.SlopedPlaneGroundProfile;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.ThreadTools;

public class LinearStickSlipGroundContactModelTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testNonlinearZForce()
   {
      boolean visualize = false;
      
      SimulationConstructionSet scs = null;
      YoVariableRegistry registry;

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestZForce"));
         registry = scs.getRootRegistry();
      }
      else
      {
         registry = new YoVariableRegistry("TestRegistry");
      }

      GroundContactPoint groundContactPoint = new GroundContactPoint("testPoint", registry);
      GroundContactPointsHolder pointsHolder = createGroundContactPointsHolder(groundContactPoint);

      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(pointsHolder, registry);
      groundContactModel.disableSlipping();

      if (visualize)
      {
         scs.startOnAThread();
      }

      for (double z = 0.00001; z>-0.02; z = z - 0.00001)
      {
         Point3d position = new Point3d(0.0, 0.0, z);
         Vector3d velocity = new Vector3d(0.0, 0.0, 0.0);

         groundContactPoint.setPosition(position);
         groundContactPoint.setVelocity(velocity);

         groundContactModel.enableSurfaceNormal();
         groundContactModel.doGroundContact();

         Vector3d force = new Vector3d();
         groundContactPoint.getForce(force);
         
         if (visualize)
         {
            scs.tickAndUpdate();
         }
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testOnFlatGroundNoSlipCompareWithAndWithoutNormals()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");

      GroundContactPoint groundContactPoint = new GroundContactPoint("testPoint", registry);
      GroundContactPointsHolder pointsHolder = createGroundContactPointsHolder(groundContactPoint);

      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(pointsHolder, registry);
      groundContactModel.disableSlipping();

      Point3d position = new Point3d(0.0, 0.0, -0.002);
      Vector3d velocity = new Vector3d(0.0, 0.0, -1.0);

      groundContactPoint.setPosition(position);
      groundContactPoint.setVelocity(velocity);

      groundContactModel.enableSurfaceNormal();
      groundContactModel.doGroundContact();

      Vector3d force = new Vector3d();
      groundContactPoint.getForce(force);

      assertEquals(0.0, force.getX(), 1e-7);
      assertEquals(0.0, force.getY(), 1e-7);
      assertTrue(force.getZ() > 0.0);

      Point3d touchdownPosition = new Point3d();
      groundContactPoint.getTouchdownLocation(touchdownPosition);

      JUnitTools.assertTuple3dEquals(touchdownPosition, position, 1e-7);

      groundContactModel.disableSurfaceNormal();
      groundContactModel.doGroundContact();

      Vector3d forceWithNormalsDisabled = new Vector3d();
      groundContactPoint.getForce(forceWithNormalsDisabled);

      JUnitTools.assertTuple3dEquals(force, forceWithNormalsDisabled, 1e-7);

      int numberOfTests = 1000;

      Random random = new Random(1977L);

      for (int i = 0; i < numberOfTests; i++)
      {
         double maxAbsoluteX = 0.01;
         double maxAbsoluteY = 0.01;
         double maxAbsoluteZ = 0.01;
         double maxSpeed = 0.1;

         position = RandomTools.generateRandomPoint(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ);

         // Keep it under ground for now to make sure touchdown doesn't change.
         if (position.getZ() > -0.002)
            position.setZ(-0.002);

         velocity = RandomTools.generateRandomVector(random, maxSpeed);
         if (velocity.getZ() > 0.0) velocity.setZ(-velocity.getZ());

         groundContactPoint.setPosition(position);
         groundContactPoint.setVelocity(velocity);

         groundContactModel.enableSurfaceNormal();
         groundContactModel.doGroundContact();
         assertTrue(groundContactPoint.isInContact());
         groundContactPoint.getForce(force);

         groundContactModel.disableSurfaceNormal();
         groundContactModel.doGroundContact();
         assertTrue(groundContactPoint.isInContact());
         groundContactPoint.getForce(forceWithNormalsDisabled);

         JUnitTools.assertTuple3dEquals(force, forceWithNormalsDisabled, 1e-7);

         Point3d touchdownTest = new Point3d();
         groundContactPoint.getTouchdownLocation(touchdownTest);

         JUnitTools.assertTuple3dEquals(touchdownPosition, touchdownTest, 1e-7);
      }

      // Test one above ground:
      position.set(0.2, 0.3, 1e-7);
      velocity.set(0.0, 0.0, 0.0);

      groundContactPoint.setPosition(position);
      groundContactPoint.setVelocity(velocity);

      groundContactModel.enableSurfaceNormal();
      groundContactModel.doGroundContact();
      assertFalse(groundContactPoint.isInContact());
      groundContactPoint.getForce(force);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, 0.0), force, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testOnSlantedGroundCompareWithAndWithoutNormals()
   {
      YoVariableRegistry registryOnFlat = new YoVariableRegistry("TestRegistryOnFlat");
      YoVariableRegistry registryOnSlope = new YoVariableRegistry("TestRegistryOnFlat");

      GroundContactPoint groundContactPointOnFlat = new GroundContactPoint("testPointOnFlat", registryOnFlat);
      GroundContactPointsHolder pointsHolderOnFlat = createGroundContactPointsHolder(groundContactPointOnFlat);
      
      GroundContactPoint groundContactPointOnSlope = new GroundContactPoint("testPointOnSlope", registryOnSlope);
      GroundContactPointsHolder pointsHolderOnSlope = createGroundContactPointsHolder(groundContactPointOnSlope);
      
      LinearStickSlipGroundContactModel groundContactModelOnFlat = new LinearStickSlipGroundContactModel(pointsHolderOnFlat, registryOnFlat);
      groundContactModelOnFlat.enableSlipping();
      
      LinearStickSlipGroundContactModel groundContactModelOnSlope = new LinearStickSlipGroundContactModel(pointsHolderOnSlope, registryOnSlope);
      groundContactModelOnSlope.enableSlipping();
      
      FlatGroundProfile flatGroundProfile = new FlatGroundProfile();
      groundContactModelOnFlat.setGroundProfile3D(flatGroundProfile);
      
      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.setRotationRollAndZeroTranslation(0.3);
      transform3D.setRotationPitchAndZeroTranslation(-0.7);
      transform3D.setTranslation(new Vector3d(0.1, 0.2, 0.3));
      
      RigidBodyTransform inverseTransform3D = new RigidBodyTransform(transform3D);
      inverseTransform3D.invert();
      
      Vector3d surfaceNormal = new Vector3d(0.0, 0.0, 1.0);
      transform3D.transform(surfaceNormal);
      surfaceNormal.normalize();
      Point3d intersectionPoint = new Point3d();
      transform3D.transform(intersectionPoint);
      
      SlopedPlaneGroundProfile slopedGroundProfile = new SlopedPlaneGroundProfile(surfaceNormal, intersectionPoint, 100.0);
      groundContactModelOnSlope.setGroundProfile3D(slopedGroundProfile);
      
      Random random = new Random(1833L);

      int numberOfTests = 10000;

      for (int i=0; i<numberOfTests; i++)
      {
         double maxAbsoluteXYZ = 0.1;
         double maxAbsoluteVelocity = 1.0;
         Point3d queryPointOnFlat = RandomTools.generateRandomPoint(random, maxAbsoluteXYZ , maxAbsoluteXYZ, maxAbsoluteXYZ);
         Vector3d queryVelocityOnFlat = RandomTools.generateRandomVector(random, maxAbsoluteVelocity);

         groundContactPointOnFlat.setPosition(queryPointOnFlat);
         groundContactPointOnFlat.setVelocity(queryVelocityOnFlat);
         groundContactModelOnFlat.doGroundContact();
         Vector3d forceOnFlat = new Vector3d();
         groundContactPointOnFlat.getForce(forceOnFlat);

         Point3d queryPointOnSlope = new Point3d(queryPointOnFlat);
         Vector3d queryVelocityOnSlope = new Vector3d(queryVelocityOnFlat);

         transform3D.transform(queryPointOnSlope);
         transform3D.transform(queryVelocityOnSlope);

         groundContactPointOnSlope.setPosition(queryPointOnSlope);
         groundContactPointOnSlope.setVelocity(queryVelocityOnSlope);
         groundContactModelOnSlope.doGroundContact();

         Vector3d forceOnSlope = new Vector3d();
         groundContactPointOnSlope.getForce(forceOnSlope);

         inverseTransform3D.transform(forceOnSlope);

         JUnitTools.assertTuple3dEquals(forceOnFlat, forceOnSlope, 1e-7);
         
         assertTrue(groundContactPointOnFlat.isInContact() == groundContactPointOnSlope.isInContact());
         assertTrue(groundContactPointOnFlat.isSlipping() == groundContactPointOnSlope.isSlipping());
         
      }
   }


   private GroundContactPointsHolder createGroundContactPointsHolder(GroundContactPoint groundContactPoint)
   {
      final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<GroundContactPoint>();
      groundContactPoints.add(groundContactPoint);

      GroundContactPointsHolder pointsHolder = new GroundContactPointsHolder()
      {
         @Override
         public ArrayList<GroundContactPoint> getGroundContactPoints(int groundContactGroupIdentifier)
         {
            return groundContactPoints;
         }
      };
      return pointsHolder;
   }

}
