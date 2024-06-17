package us.ihmc.footstepPlanning.graphSearch.collision;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

public class BoundingBoxCollisionDetectorTest
{
   @Test
   public void testBodyCollisionWithZeroYaw()
   {
      DefaultFootstepPlannerParametersReadOnly plannerParameters = new DefaultFootstepPlannerParameters();
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxDepth(1.0);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxWidth(0.5);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxHeight(1.0);

      double boxDepth = plannerParameters.getBodyBoxDepth();
      double boxWidth = plannerParameters.getBodyBoxWidth();
      double boxHeight = plannerParameters.getBodyBoxHeight();
      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector();
      collisionChecker.setBoxDimensions(boxDepth, boxWidth, boxHeight);
      collisionChecker.setBoxPose(0.25, 0.5, 0.0, 0.0);
      double distanceFromBox = 1e-3;

      // test slightly outside along y
      PlanarRegionsList planarRegionsList = getSquarePlanarRegionsList(0.0, -distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      BodyCollisionData collisionData = collisionChecker.checkForCollision();
      assertFalse(collisionData.isCollisionDetected());

      planarRegionsList = getSquarePlanarRegionsList(0.5, 1.0 + distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertFalse(collisionData.isCollisionDetected());

      // test slightly outside along x
      planarRegionsList = getSquarePlanarRegionsList(1.0 + distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertFalse(collisionData.isCollisionDetected());

      planarRegionsList = getSquarePlanarRegionsList(-0.5 - distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertFalse(collisionData.isCollisionDetected());

      // test slightly inside along y
      planarRegionsList = getSquarePlanarRegionsList(0.0, distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertTrue(collisionData.isCollisionDetected());

      planarRegionsList = getSquarePlanarRegionsList(0.5, 1.0 - distanceFromBox, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertTrue(collisionData.isCollisionDetected());

      // test slightly inside along x
      planarRegionsList = getSquarePlanarRegionsList(1.0 - distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertTrue(collisionData.isCollisionDetected());

      planarRegionsList = getSquarePlanarRegionsList(-0.5 + distanceFromBox, 0.5, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      collisionData = collisionChecker.checkForCollision();
      assertTrue(collisionData.isCollisionDetected());
   }

   @Test
   public void testCollisionWithRotatedBody()
   {
      DefaultFootstepPlannerParametersReadOnly plannerParameters = new DefaultFootstepPlannerParameters();
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxDepth(0.5);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxWidth(1.0);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxHeight(1.0);

      double boxDepth = plannerParameters.getBodyBoxDepth();
      double boxWidth = plannerParameters.getBodyBoxWidth();
      double boxHeight = plannerParameters.getBodyBoxHeight();
      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector();
      collisionChecker.setBoxDimensions(boxDepth, boxWidth, boxHeight);

      double distanceFromRegionAtZeroYaw = 0.025;
      PlanarRegionsList planarRegionsList = getSquarePlanarRegionsList(0.75, 0.75, 0.5, 0.0, 0.5 - 2.0 * distanceFromRegionAtZeroYaw);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      // assert no collision at zero yaw
      collisionChecker.setBoxPose(0.25, 0.25, 0.0, Math.toRadians(0.0));
      BodyCollisionData collisionData = collisionChecker.checkForCollision();
      assertFalse(collisionData.isCollisionDetected());

      // assert no collision at small positive rotation
      collisionChecker.setBoxPose(0.25, 0.25, 0.0, Math.toRadians(15.0));
      collisionData = collisionChecker.checkForCollision();
      assertFalse(collisionData.isCollisionDetected());

      // assert collision at small negative rotation
      collisionChecker.setBoxPose(0.25, 0.25, 0.0, Math.toRadians(-15.0));
      collisionData = collisionChecker.checkForCollision();
      assertTrue(collisionData.isCollisionDetected());
   }

   @Test
   public void testHeightDetection()
   {
      DefaultFootstepPlannerParametersReadOnly plannerParameters = new DefaultFootstepPlannerParameters();
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxDepth(0.5);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxWidth(1.0);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxHeight(1.0);

      double boxDepth = plannerParameters.getBodyBoxDepth();
      double boxWidth = plannerParameters.getBodyBoxWidth();
      double boxHeight = plannerParameters.getBodyBoxHeight();
      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector();
      collisionChecker.setBoxDimensions(boxDepth, boxWidth, boxHeight);

      PlanarRegionsList planarRegionsList = getSquarePlanarRegionsList(0.25, 0.25, 0.5, 0.0, 0.5);
      collisionChecker.setPlanarRegionsList(planarRegionsList);

      // check when body is too high
      collisionChecker.setBoxPose(0.0, 0.0, 0.51, 0.0);
      assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // check when body is just below too high
      collisionChecker.setBoxPose(0.0, 0.0, 0.49, 0.0);
      assertTrue(collisionChecker.checkForCollision().isCollisionDetected());

      // check when body is too low
      collisionChecker.setBoxPose(0.0, 0.0, -0.51, 0.0);
      assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // check when body is just above too low
      collisionChecker.setBoxPose(0.0, 0.0, -0.49, 0.0);
      assertTrue(collisionChecker.checkForCollision().isCollisionDetected());
   }

   @Test
   public void testCollidingWithACube()
   {
      DefaultFootstepPlannerParametersReadOnly plannerParameters = new DefaultFootstepPlannerParameters();
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxDepth(0.5);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxWidth(1.0);
      ((DefaultFootstepPlannerParameters) plannerParameters).setBodyBoxHeight(1.0);

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

      double boxDepth = plannerParameters.getBodyBoxDepth();
      double boxWidth = plannerParameters.getBodyBoxWidth();
      double boxHeight = plannerParameters.getBodyBoxHeight();
      BoundingBoxCollisionDetector collisionChecker = new BoundingBoxCollisionDetector();
      collisionChecker.setBoxDimensions(boxDepth, boxWidth, boxHeight);

      collisionChecker.setPlanarRegionsList(planarRegionsList);

      // test just outside along y
      collisionChecker.setBoxPose(0.25, 0.01, 0.0, Math.toRadians(90.0));
      assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // test just inside along y
      collisionChecker.setBoxPose(0.15, -0.01, 0.0, Math.toRadians(90.0));
      assertTrue(collisionChecker.checkForCollision().isCollisionDetected());

      // test just outside along x
      collisionChecker.setBoxPose(-0.75 - 0.01, -0.55, 0.0, Math.toRadians(90.0));
      assertFalse(collisionChecker.checkForCollision().isCollisionDetected());

      // test just inside along x
      collisionChecker.setBoxPose(-0.75 + 0.01, -0.45, 0.0, Math.toRadians(90.0));
      assertTrue(collisionChecker.checkForCollision().isCollisionDetected());
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
      transformToWorld.getRotation().setToYawOrientation(yaw);
      return new PlanarRegionsList(new PlanarRegion(transformToWorld, Arrays.asList(unitSquare)));
   }
}
