package us.ihmc.simulationconstructionset.util.ground;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionTerrainObjectTest
{
   private static final double DEFAULT_ALLOWABLE_PENETRATION_THICKNESS = 1e5;

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testHeightAt() throws Exception
   {
      Random random = new Random(1776L);

      for (int i = 0; i < 100000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10),
                                                                                                            RandomNumbers.nextDouble(random, 30.0),
                                                                                                            random.nextInt(10));
         BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

         double randomXCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinX() - 10.0, boundingBox3dInWorld.getMaxX() + 10.0);
         double randomYCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinY() - 10.0, boundingBox3dInWorld.getMaxY() + 10.0);
         double randomZCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinZ() - 10.0, boundingBox3dInWorld.getMaxZ() + 10.0);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         double planarRegionZAtXY = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord);

         if (planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {
            assertEquals(planarRegionZAtXY, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-10);
         }
         else
         {
            assertNotEquals(planarRegionZAtXY, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testHeightAndNormalAt() throws Exception
   {
      Random random = new Random(1776L);

      Vector3D terrainObjectNormalToPack = new Vector3D();
      Vector3D planarRegionNormalToPack = new Vector3D();
      Vector3D standardGroundNormal = new Vector3D(0.0, 0.0, 1.0);

      for (int i = 0; i < 100000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10),
                                                                                                            RandomNumbers.nextDouble(random, 30.0),
                                                                                                            random.nextInt(10));
         BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

         double randomXCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinX() - 10.0, boundingBox3dInWorld.getMaxX() + 10.0);
         double randomYCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinY() - 10.0, boundingBox3dInWorld.getMaxY() + 10.0);
         double randomZCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinZ() - 10.0, boundingBox3dInWorld.getMaxZ() + 10.0);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         double planarRegionZAtXY = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord);
         double heightAt = terrainObject.heightAndNormalAt(randomXCoord, randomYCoord, randomZCoord, terrainObjectNormalToPack);
         planarRegion.getNormal(planarRegionNormalToPack);

         if (planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {
            assertEquals(planarRegionZAtXY, heightAt, 1e-10);
            EuclidCoreTestTools.assertTuple3DEquals("Normals are not equal!", terrainObjectNormalToPack, planarRegionNormalToPack, 1e-10);
         }
         else
         {
            assertNotEquals(planarRegionZAtXY, heightAt, 1e-10);
            EuclidCoreTestTools.assertTuple3DEquals("Normals are not equal!", standardGroundNormal, terrainObjectNormalToPack, 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test //(timeout = 30000)
   public void testGetBoundingBox() throws Exception
   {
      Random random = new Random(1776L);

      Point3D planarRegionBoundingBoxMinPoint = new Point3D();
      Point3D planarRegionBoundingBoxMaxPoint = new Point3D();
      Point3D terrainObjectBoundingBoxMinPoint = new Point3D();
      Point3D terrainObjectBoundingBoxMaxPoint = new Point3D();

      for (int i = 0; i < 100000; i++)
      {
         int numberOfRandomlyGeneratedPolygons = random.nextInt(10);
         double maxAbsoluteXYForPolygons = RandomNumbers.nextDouble(random, 30.0);
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, numberOfRandomlyGeneratedPolygons,
                                                                                                            maxAbsoluteXYForPolygons,
                                                                                                            numberOfRandomlyGeneratedPolygons);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         planarRegion.getBoundingBox3dInWorld().getMinPoint(planarRegionBoundingBoxMinPoint);
         planarRegion.getBoundingBox3dInWorld().getMaxPoint(planarRegionBoundingBoxMaxPoint);
         terrainObject.getBoundingBox().getMinPoint(terrainObjectBoundingBoxMinPoint);
         terrainObject.getBoundingBox().getMaxPoint(terrainObjectBoundingBoxMaxPoint);

         if (numberOfRandomlyGeneratedPolygons == 0)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(planarRegionBoundingBoxMinPoint);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(planarRegionBoundingBoxMaxPoint);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(terrainObjectBoundingBoxMinPoint);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(terrainObjectBoundingBoxMaxPoint);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals("Bounding box min points are not equal!", planarRegionBoundingBoxMinPoint, terrainObjectBoundingBoxMinPoint,
                                                    1e-10);
            EuclidCoreTestTools.assertTuple3DEquals("Bounding box max points are not equal!", planarRegionBoundingBoxMaxPoint, terrainObjectBoundingBoxMaxPoint,
                                                    1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testIsClose() throws Exception
   {
      Random random = new Random(1776L);

      for (int i = 0; i < 100000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10),
                                                                                                            RandomNumbers.nextDouble(random, 30.0),
                                                                                                            random.nextInt(10));
         BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

         double randomXCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinX() - 10.0, boundingBox3dInWorld.getMaxX() + 10.0);
         double randomYCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinY() - 10.0, boundingBox3dInWorld.getMaxY() + 10.0);
         double randomZCoord = RandomNumbers.nextDouble(random, boundingBox3dInWorld.getMinZ() - 10.0, boundingBox3dInWorld.getMaxZ() + 10.0);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         if (boundingBox3dInWorld.isXYInsideInclusive(randomXCoord, randomYCoord))
         {
            assertTrue(terrainObject.isClose(randomXCoord, randomYCoord, randomZCoord));
         }
         else
         {
            assertFalse(terrainObject.isClose(randomXCoord, randomYCoord, randomZCoord));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testCheckIfInside() throws Exception
   {
      Random random = new Random(1776L);
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      Vector3D translation = new Vector3D();
      Point3D randomPoint = new Point3D();
      Vector3D planarRegionSurfaceNormal = new Vector3D();

      Vector3D terrainObjectNormalToPack = new Vector3D();
      Point3D terrainObjectIntersectionToPack = new Point3D();

      for (int i = 0; i < 100000; i++)
      {

         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10),
                                                                                                            RandomNumbers.nextDouble(random, 30.0),
                                                                                                            random.nextInt(10));

         double randomXCoord = RandomNumbers.nextDouble(random, 15.0);
         double randomYCoord = RandomNumbers.nextDouble(random, 15.0);
         double randomZCoord;

         boolean shouldGeneratePointGuaranteedOnPlane = random.nextBoolean();

         if (shouldGeneratePointGuaranteedOnPlane && planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {
            randomZCoord = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord) + RandomNumbers.nextDouble(random, 1e-8);
         }
         else
         {
            randomZCoord = RandomNumbers.nextDouble(random, 15.0);
         }

         randomPoint.set(randomXCoord, randomYCoord, randomZCoord);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         planarRegion.getTransformToWorld(transformToWorld);
         transformToWorld.getTranslation(translation);
         planarRegion.getNormal(planarRegionSurfaceNormal);

         if (planarRegion.isPointOnOrSlightlyBelow(randomPoint, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS))
         {
            assertTrue(terrainObject.checkIfInside(randomXCoord, randomYCoord, randomZCoord, terrainObjectIntersectionToPack, terrainObjectNormalToPack));
            EuclidCoreTestTools.assertTuple3DEquals("Intersection to pack is not correct!", randomPoint, terrainObjectIntersectionToPack, 1e-10);
            EuclidCoreTestTools.assertTuple3DEquals("Surface normal to pack is not correct!", planarRegionSurfaceNormal, terrainObjectNormalToPack, 1e-10);
         }
         else
         {
            assertFalse(terrainObject.checkIfInside(randomXCoord, randomYCoord, randomZCoord, null, null));
         }
      }
   }
}