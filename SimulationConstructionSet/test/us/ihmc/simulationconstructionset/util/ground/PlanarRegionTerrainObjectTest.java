package us.ihmc.simulationconstructionset.util.ground;

import org.junit.Test;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.Random;

import static org.junit.Assert.*;

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
         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomTools.generateRandomDouble(random, 30.0),
                     random.nextInt(10));
         BoundingBox3d boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

         double randomXCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getXMin() - 10.0, boundingBox3dInWorld.getXMax() + 10.0);
         double randomYCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getYMin() - 10.0, boundingBox3dInWorld.getYMax() + 10.0);
         double randomZCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getZMin() - 10.0, boundingBox3dInWorld.getZMax() + 10.0);

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

      Vector3d terrainObjectNormalToPack = new Vector3d();
      Vector3d planarRegionNormalToPack = new Vector3d();
      Vector3d standardGroundNormal = new Vector3d(0.0, 0.0, 1.0);

      for (int i = 0; i < 100000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomTools.generateRandomDouble(random, 30.0),
                     random.nextInt(10));
         BoundingBox3d boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

         double randomXCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getXMin() - 10.0, boundingBox3dInWorld.getXMax() + 10.0);
         double randomYCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getYMin() - 10.0, boundingBox3dInWorld.getYMax() + 10.0);
         double randomZCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getZMin() - 10.0, boundingBox3dInWorld.getZMax() + 10.0);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         double planarRegionZAtXY = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord);
         double heightAt = terrainObject.heightAndNormalAt(randomXCoord, randomYCoord, randomZCoord, terrainObjectNormalToPack);
         planarRegion.getNormal(planarRegionNormalToPack);

         if (planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {
            assertEquals(planarRegionZAtXY, heightAt, 1e-10);
            JUnitTools.assertVector3dEquals("Normals are not equal!", terrainObjectNormalToPack, planarRegionNormalToPack, 1e-10);
         }
         else
         {
            assertNotEquals(planarRegionZAtXY, heightAt, 1e-10);
            JUnitTools.assertVector3dEquals("Normals are not equal!", standardGroundNormal, terrainObjectNormalToPack, 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testGetBoundingBox() throws Exception
   {
      Random random = new Random(1776L);

      Point3d planarRegionBoundingBoxMinPoint = new Point3d();
      Point3d planarRegionBoundingBoxMaxPoint = new Point3d();
      Point3d terrainObjectBoundingBoxMinPoint = new Point3d();
      Point3d terrainObjectBoundingBoxMaxPoint = new Point3d();

      for (int i = 0; i < 100000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomTools.generateRandomDouble(random, 30.0),
                     random.nextInt(10));

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         planarRegion.getBoundingBox3dInWorld().getMinPoint(planarRegionBoundingBoxMinPoint);
         planarRegion.getBoundingBox3dInWorld().getMaxPoint(planarRegionBoundingBoxMaxPoint);
         terrainObject.getBoundingBox().getMinPoint(terrainObjectBoundingBoxMinPoint);
         terrainObject.getBoundingBox().getMaxPoint(terrainObjectBoundingBoxMaxPoint);

         JUnitTools.assertPoint3dEquals("Bounding box min points are not equal!", planarRegionBoundingBoxMinPoint, terrainObjectBoundingBoxMinPoint, 1e-10);
         JUnitTools.assertPoint3dEquals("Bounding box max points are not equal!", planarRegionBoundingBoxMaxPoint, terrainObjectBoundingBoxMaxPoint, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testIsClose() throws Exception
   {
      Random random = new Random(1776L);

      for (int i = 0; i < 100000; i++)
      {
         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomTools.generateRandomDouble(random, 30.0),
                     random.nextInt(10));
         BoundingBox3d boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

         double randomXCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getXMin() - 10.0, boundingBox3dInWorld.getXMax() + 10.0);
         double randomYCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getYMin() - 10.0, boundingBox3dInWorld.getYMax() + 10.0);
         double randomZCoord = RandomTools.generateRandomDouble(random, boundingBox3dInWorld.getZMin() - 10.0, boundingBox3dInWorld.getZMax() + 10.0);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         if (boundingBox3dInWorld.isXYInside(randomXCoord, randomYCoord))
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
      Vector3d translation = new Vector3d();
      Point3d randomPoint = new Point3d();
      Vector3d planarRegionSurfaceNormal = new Vector3d();

      Vector3d terrainObjectNormalToPack = new Vector3d();
      Point3d terrainObjectIntersectionToPack = new Point3d();

      for (int i = 0; i < 100000; i++)
      {

         PlanarRegion planarRegion = PlanarRegion
               .generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, random.nextInt(10), RandomTools.generateRandomDouble(random, 30.0),
                     random.nextInt(10));

         double randomXCoord = RandomTools.generateRandomDouble(random, 15.0);
         double randomYCoord = RandomTools.generateRandomDouble(random, 15.0);
         double randomZCoord;

         boolean shouldGeneratePointGuaranteedOnPlane = random.nextBoolean();

         if (shouldGeneratePointGuaranteedOnPlane && planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {
            randomZCoord = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord) + RandomTools.generateRandomDouble(random, 1e-8);
         }
         else
         {
            randomZCoord = RandomTools.generateRandomDouble(random, 15.0);
         }

         randomPoint.set(randomXCoord, randomYCoord, randomZCoord);

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS);

         planarRegion.getTransformToWorld(transformToWorld);
         transformToWorld.getTranslation(translation);
         planarRegion.getNormal(planarRegionSurfaceNormal);

         if (planarRegion.isPointOnOrSlightlyBelow(randomPoint, DEFAULT_ALLOWABLE_PENETRATION_THICKNESS))
         {
            assertTrue(terrainObject.checkIfInside(randomXCoord, randomYCoord, randomZCoord, terrainObjectIntersectionToPack, terrainObjectNormalToPack));
            JUnitTools.assertPoint3dEquals("Intersection to pack is not correct!", randomPoint, terrainObjectIntersectionToPack, 1e-10);
            JUnitTools.assertVector3dEquals("Surface normal to pack is not correct!", planarRegionSurfaceNormal, terrainObjectNormalToPack, 1e-10);
         }
         else
         {
            assertFalse(terrainObject.checkIfInside(randomXCoord, randomYCoord, randomZCoord, null, null));
         }
      }
   }
}