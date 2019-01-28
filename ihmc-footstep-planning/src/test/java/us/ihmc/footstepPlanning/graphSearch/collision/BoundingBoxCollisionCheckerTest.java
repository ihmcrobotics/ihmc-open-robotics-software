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

import java.util.Arrays;

public class BoundingBoxCollisionCheckerTest
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

      BoundingBoxCollisionChecker collisionChecker = new BoundingBoxCollisionChecker(plannerParameters);
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
