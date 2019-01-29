package us.ihmc.footstepPlanning.graphSearch.collision;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import java.util.Arrays;

public class BoundingBoxCollisionDetectorTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBodyCollisionWithZeroYaw()
   {
      FootstepPlannerParameters plannerParameters = new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getBodyBoxDepth()
         {
            return 1.0;
         }

         @Override
         public double getBodyBoxWidth()
         {
            return 0.5;
         }

         @Override
         public double getBodyBoxHeight()
         {
            return 1.0;
         }

         @Override
         public FootstepPlannerCostParameters getCostParameters()
         {
            return new DefaultFootstepPlannerCostParameters()
            {
               @Override
               public double getMaximum2dDistanceFromBoundingBoxToPenalize()
               {
                  return 0.25;
               }
            };
         }
      };

      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector(plannerParameters);
      collisionChecker.setBoxPose(0.25, 0.5, 0.0, 0.0);
      double distanceFromBox = 1e-3;

      // test slightly outside along y
      PlanarRegionsList planarRegionsList = getSquarePlanarRegionsList(0.0, -distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      BodyCollisionData collisionData = collisionChecker.checkForCollision();
      Assert.assertFalse(collisionData.isCollisionDetected());
      Assert.assertEquals(collisionData.getDistanceFromBoundingBox(), distanceFromBox, 1e-8);

      planarRegionsList = getSquarePlanarRegionsList(0.5, 1.0 + distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertFalse(collisionData.isCollisionDetected());
      Assert.assertEquals(collisionData.getDistanceFromBoundingBox(), distanceFromBox, 1e-8);

      // test slightly outside along x
      planarRegionsList = getSquarePlanarRegionsList(1.0 + distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertFalse(collisionData.isCollisionDetected());
      Assert.assertEquals(collisionData.getDistanceFromBoundingBox(), distanceFromBox, 1e-8);

      planarRegionsList = getSquarePlanarRegionsList(-0.5 - distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertFalse(collisionData.isCollisionDetected());
      Assert.assertEquals(collisionData.getDistanceFromBoundingBox(), distanceFromBox, 1e-8);

      // test slightly inside along y
      planarRegionsList = getSquarePlanarRegionsList(0.0, distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertTrue(collisionData.isCollisionDetected());

      planarRegionsList = getSquarePlanarRegionsList(0.5, 1.0 - distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertTrue(collisionData.isCollisionDetected());

      // test slightly inside along x
      planarRegionsList = getSquarePlanarRegionsList(1.0 - distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertTrue(collisionData.isCollisionDetected());

      planarRegionsList = getSquarePlanarRegionsList(-0.5 + distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      Assert.assertTrue(collisionData.isCollisionDetected());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCollisionWithRotatedBody()
   {
      FootstepPlannerParameters plannerParameters = new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getBodyBoxDepth()
         {
            return 0.5;
         }

         @Override
         public double getBodyBoxWidth()
         {
            return 1.0;
         }

         @Override
         public double getBodyBoxHeight()
         {
            return 1.0;
         }

         @Override
         public FootstepPlannerCostParameters getCostParameters()
         {
            return new DefaultFootstepPlannerCostParameters()
            {
               @Override
               public double getMaximum2dDistanceFromBoundingBoxToPenalize()
               {
                  return 0.25;
               }
            };
         }
      };

      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector(plannerParameters);
      double distanceFromRegionAtZeroYaw = 0.025;
      PlanarRegionsList planarRegionsList = getSquarePlanarRegionsList(0.75, 0.75, 0.5, 0.0, 0.5 - 2.0 * distanceFromRegionAtZeroYaw);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      // assert no collision at zero yaw
      collisionChecker.setBoxPose(0.25, 0.25, 0.0, Math.toRadians(0.0));
      BodyCollisionData collisionData = collisionChecker.checkForCollision();
      Assert.assertFalse(collisionData.isCollisionDetected());
      Assert.assertEquals(collisionData.getDistanceFromBoundingBox(), distanceFromRegionAtZeroYaw, 1e-8);

      // assert no collision at small positive rotation
      collisionChecker.setBoxPose(0.25, 0.25, 0.0, Math.toRadians(15.0));
      collisionData = collisionChecker.checkForCollision();
      Assert.assertFalse(collisionData.isCollisionDetected());

      // assert collision at small negative rotation
      collisionChecker.setBoxPose(0.25, 0.25, 0.0, Math.toRadians(-15.0));
      collisionData = collisionChecker.checkForCollision();
      Assert.assertTrue(collisionData.isCollisionDetected());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHeightDetection()
   {
      FootstepPlannerParameters plannerParameters = new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getBodyBoxDepth()
         {
            return 0.5;
         }

         @Override
         public double getBodyBoxWidth()
         {
            return 1.0;
         }

         @Override
         public double getBodyBoxHeight()
         {
            return 1.0;
         }

         @Override
         public FootstepPlannerCostParameters getCostParameters()
         {
            return new DefaultFootstepPlannerCostParameters()
            {
               @Override
               public double getMaximum2dDistanceFromBoundingBoxToPenalize()
               {
                  return 0.25;
               }
            };
         }
      };

      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector(plannerParameters);
      PlanarRegionsList planarRegionsList = getSquarePlanarRegionsList(0.25, 0.25, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      // check when body is too high
      collisionChecker.setBoxPose(0.0, 0.0, 0.51, 0.0);
      Assert.assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // check when body is just below too high
      collisionChecker.setBoxPose(0.0, 0.0, 0.49, 0.0);
      Assert.assertTrue(collisionChecker.checkForCollision().isCollisionDetected());

      // check when body is too low
      collisionChecker.setBoxPose(0.0, 0.0, -0.51, 0.0);
      Assert.assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // check when body is just above too low
      collisionChecker.setBoxPose(0.0, 0.0, -0.49, 0.0);
      Assert.assertTrue(collisionChecker.checkForCollision().isCollisionDetected());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCollidingWithACube()
   {
      FootstepPlannerParameters plannerParameters = new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getBodyBoxDepth()
         {
            return 0.5;
         }

         @Override
         public double getBodyBoxWidth()
         {
            return 1.0;
         }

         @Override
         public double getBodyBoxHeight()
         {
            return 1.0;
         }

         @Override
         public FootstepPlannerCostParameters getCostParameters()
         {
            return new DefaultFootstepPlannerCostParameters()
            {
               @Override
               public double getMaximum2dDistanceFromBoundingBoxToPenalize()
               {
                  return 0.25;
               }
            };
         }
      };

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double cubeX = 0.25;
      double cubeY = -0.5;
      double cubeZ = -0.25;
      generator.translate(cubeX, cubeY, cubeZ);

      double cubeDepth = 1.0;
      double cubeWidth = 0.5;
      double cubeHeight = 1.0;
      generator.addCubeReferencedAtBottomMiddle(cubeDepth, cubeWidth, cubeHeight);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector(plannerParameters);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      // test just outside along y
      collisionChecker.setBoxPose(0.25, 0.01, 0.0, Math.toRadians(90.0));
      Assert.assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // test just inside along y
      collisionChecker.setBoxPose(0.15, -0.01, 0.0, Math.toRadians(90.0));
      Assert.assertTrue(collisionChecker.checkForCollision().isCollisionDetected());

      // test just outside along x
      collisionChecker.setBoxPose(-0.75 - 0.01, -0.55, 0.0, Math.toRadians(90.0));
      Assert.assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // test just inside along x
      collisionChecker.setBoxPose(-0.75 + 0.01, -0.45, 0.0, Math.toRadians(90.0));
      Assert.assertTrue(collisionChecker.checkForCollision().isCollisionDetected());
   }

   private static PlanarRegionsList getSquarePlanarRegionsList(double x, double y, double z, double yaw, double sideLength)
   {
      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(0.5 * sideLength, 0.5 * sideLength);
      unitSquare.addVertex(-0.5 * sideLength, 0.5 * sideLength);
      unitSquare.addVertex(0.5 * sideLength, -0.5 * sideLength);
      unitSquare.addVertex(-0.5 * sideLength, -0.5 * sideLength);
      unitSquare.update();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslationAndIdentityRotation(x, y, z);
      transformToWorld.setRotationYaw(yaw);
      return new PlanarRegionsList(new PlanarRegion(transformToWorld, Arrays.asList(unitSquare)));
   }
}
