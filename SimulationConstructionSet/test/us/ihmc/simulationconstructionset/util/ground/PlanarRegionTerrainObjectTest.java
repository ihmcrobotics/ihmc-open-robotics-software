package us.ihmc.simulationconstructionset.util.ground;

import org.junit.Test;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionTerrainObjectTest
{
   @Test
   public void testHeightAt() throws Exception
   {
      Random random = new Random(1776L);
      final double zLocationOfPlanarRegion = 2.0;
      Point3d minPoint = new Point3d(-1.0, -1.0, zLocationOfPlanarRegion);
      Point3d maxPoint = new Point3d(1.0, 1.0, zLocationOfPlanarRegion);

      List<ConvexPolygon2d> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d polygon1 = new ConvexPolygon2d();
      polygon1.addVertex(minPoint.x, minPoint.y);
      polygon1.addVertex(maxPoint.x, minPoint.y);
      polygon1.addVertex(minPoint.x, maxPoint.y);
      polygon1.addVertex(maxPoint.x, maxPoint.y);

      regionConvexPolygons.add(polygon1);

      for (ConvexPolygon2d convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      Vector3d randomTranslation = new Vector3d(0, 0, zLocationOfPlanarRegion);
      Quat4d randomOrientation = RandomTools.generateRandomQuaternion(random, Math.toRadians(45.0));
      RigidBodyTransform regionTransform = new RigidBodyTransform(randomOrientation, randomTranslation);

      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion);

      double xMin = planarRegion.getBoundingBox3dInWorld().getXMin();
      double xMax = planarRegion.getBoundingBox3dInWorld().getXMax();
      double yMin = planarRegion.getBoundingBox3dInWorld().getYMin();
      double yMax = planarRegion.getBoundingBox3dInWorld().getYMax();

      for(int i = 0; i < 1000000; i ++)
      {
         double randomXCoord = RandomTools.generateRandomDoubleInRange(random, xMin, xMax);
         double randomYCoord = RandomTools.generateRandomDoubleInRange(random, yMin, yMax);
         double randomZCoord = random.nextGaussian();

         double planarRegionZAtXY = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord);

         if(planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {

            assertEquals(planarRegionZAtXY, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-8);
         }
         else
         {
            assertNotEquals(planarRegionZAtXY, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-8);
         }
      }
   }

   @Test
   public void testHeightAndNormalAt() throws Exception
   {
      Random random = new Random(1776L);
      final double zLocationOfPlanarRegion = 2.0;
      Point3d minPoint = new Point3d(-1.0, -1.0, zLocationOfPlanarRegion);
      Point3d maxPoint = new Point3d(1.0, 1.0, zLocationOfPlanarRegion);

      List<ConvexPolygon2d> regionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d polygon1 = new ConvexPolygon2d();
      polygon1.addVertex(minPoint.x, minPoint.y);
      polygon1.addVertex(maxPoint.x, minPoint.y);
      polygon1.addVertex(minPoint.x, maxPoint.y);
      polygon1.addVertex(maxPoint.x, maxPoint.y);

      regionConvexPolygons.add(polygon1);

      for (ConvexPolygon2d convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      Vector3d randomTranslation = new Vector3d(0, 0, zLocationOfPlanarRegion);
      Quat4d randomOrientation = RandomTools.generateRandomQuaternion(random, Math.toRadians(45.0));
      RigidBodyTransform regionTransform = new RigidBodyTransform(randomOrientation, randomTranslation);

      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion);

      Vector3d terrainObjectNormalToPack = new Vector3d();
      Vector3d planarRegionNormalToPack = new Vector3d();
      Vector3d standardGroundNormal = new Vector3d(0.0, 0.0, 1.0);

      double xMin = planarRegion.getBoundingBox3dInWorld().getXMin();
      double xMax = planarRegion.getBoundingBox3dInWorld().getXMax();
      double yMin = planarRegion.getBoundingBox3dInWorld().getYMin();
      double yMax = planarRegion.getBoundingBox3dInWorld().getYMax();

      for(int i = 0; i < 1000000; i ++)
      {
         double randomXCoord = RandomTools.generateRandomDoubleInRange(random, xMin, xMax);
         double randomYCoord = RandomTools.generateRandomDoubleInRange(random, yMin, yMax);
         double randomZCoord = random.nextGaussian();

         double planarRegionZAtXY = planarRegion.getPlaneZGivenXY(randomXCoord, randomYCoord);
         double heightAt = terrainObject.heightAndNormalAt(randomXCoord, randomYCoord, randomZCoord, terrainObjectNormalToPack);
         planarRegion.getNormal(planarRegionNormalToPack);

         if(planarRegion.isPointInsideByProjectionOntoXYPlane(randomXCoord, randomYCoord))
         {
            assertEquals(planarRegionZAtXY, heightAt, 1e-8);
            assertEquals(terrainObjectNormalToPack, planarRegionNormalToPack);
         }
         else
         {
            assertNotEquals(planarRegionZAtXY, heightAt, 1e-8);
            assertEquals(terrainObjectNormalToPack, standardGroundNormal);
         }
      }
   }

   @Test
   public void testGetBoundingBox() throws Exception
   {
      fail();
   }

   @Test
   public void testGetLinkGraphics() throws Exception
   {
      fail();
   }

   @Test
   public void testIsClose() throws Exception
   {
      fail();
   }

   @Test
   public void testCheckIfInside() throws Exception
   {
      fail();
   }

   @Test
   public void testGetHeightMapIfAvailable() throws Exception
   {
      fail();
   }

}