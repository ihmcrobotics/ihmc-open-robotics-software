package us.ihmc.simulationconstructionset.util.ground;

import org.junit.Test;
import us.ihmc.robotics.geometry.PlanarRegion;
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

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion);

         double xMin = planarRegion.getBoundingBox3dInWorld().getXMin();
         double xMax = planarRegion.getBoundingBox3dInWorld().getXMax();
         double yMin = planarRegion.getBoundingBox3dInWorld().getYMin();
         double yMax = planarRegion.getBoundingBox3dInWorld().getYMax();

         double randomXCoord = RandomTools.generateRandomDoubleInRange(random, xMin, xMax);
         double randomYCoord = RandomTools.generateRandomDoubleInRange(random, yMin, yMax);
         double randomZCoord = random.nextGaussian();

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

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion);

         double xMin = planarRegion.getBoundingBox3dInWorld().getXMin();
         double xMax = planarRegion.getBoundingBox3dInWorld().getXMax();
         double yMin = planarRegion.getBoundingBox3dInWorld().getYMin();
         double yMax = planarRegion.getBoundingBox3dInWorld().getYMax();

         double randomXCoord = RandomTools.generateRandomDoubleInRange(random, xMin, xMax);
         double randomYCoord = RandomTools.generateRandomDoubleInRange(random, yMin, yMax);
         double randomZCoord = random.nextGaussian();

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

         PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion);

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
      fail();
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.0)
   @Test(timeout = 30000)
   public void testCheckIfInside() throws Exception
   {
      fail();
   }
}