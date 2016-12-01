package us.ihmc.simulationconstructionset.util.ground;

import org.junit.Test;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;

import javax.vecmath.Point3d;
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

      RigidBodyTransform regionTransform = new RigidBodyTransform();
      regionTransform.applyTranslation(new Vector3d(0, 0, zLocationOfPlanarRegion));
      PlanarRegion planarRegion = new PlanarRegion(regionTransform, regionConvexPolygons);

      PlanarRegionTerrainObject terrainObject = new PlanarRegionTerrainObject(planarRegion);

      for(int i = 0; i < 100000; i ++)
      {
         double randomXCoord = RandomTools.generateRandomDoubleInRange(random, -1.0, 1.0);
         double randomYCoord = RandomTools.generateRandomDoubleInRange(random, -1.0, 1.0);
         double randomZCoord = random.nextGaussian();

         assertEquals(2.0, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-8);

         randomXCoord = RandomTools.generateRandomDoubleInRange(random, -30.0, -1.1);
         randomYCoord = RandomTools.generateRandomDoubleInRange(random, -30.0, -1.1);

         assertNotEquals(2.0, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-8);

         randomXCoord = RandomTools.generateRandomDoubleInRange(random, 1.1, 30.0);
         randomYCoord = RandomTools.generateRandomDoubleInRange(random, 1.1, 30.0);

         assertNotEquals(2.0, terrainObject.heightAt(randomXCoord, randomYCoord, randomZCoord), 1e-8);
      }
   }

   @Test
   public void testHeightAndNormalAt() throws Exception
   {
      fail();
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