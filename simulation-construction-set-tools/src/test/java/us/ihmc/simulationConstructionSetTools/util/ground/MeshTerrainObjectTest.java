package us.ihmc.simulationConstructionSetTools.util.ground;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject.IntersectionResult;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;


/**
 * This class tests the MeshTerrainObject Class and it mainly uses two meshes to test objects
 * <p>
 * The meshObjects can be found at this link 
 * 
 * LEGO Block (CAD Model:https://cad.onshape.com/documents/618f39b230922ef75ccbda07/w/1cfb27e10c4a7a5b0e6e57a9/e/159fe516a3c575efc6c9925e )
 * Y axis has a universal width of 0.5m 
 *
 *         Z AXIS  
 *         ^  
 *         |  
 * AAAAAAAA|.......
 * AAAAAAAA|.......____ -Y Axis
 * AAAAAAAA|CCCCCCC
 * AAAAAAAA|CCCCCCC
 * ,,,,,,,,|CCCCCCC
 * ,,,,,,,,|CCCCCCC
 * 
 * HSHAPE (CAD Model:https://cad.onshape.com/documents/618f39b230922ef75ccbda07/w/1cfb27e10c4a7a5b0e6e57a9/e/a6ff244d83f3181ae8497374 )
 *       Z AXIS  
 *       ^  
 *       |  
 * AAAA,,,,CCCC
 * AAAA,,,,CCCC
 * AAAABBBBCCCC___ X Axis
 * AAAABBBBCCCC
 * AAAA....CCCC
 * AAAA....CCCC
 * 
 * </p>
 * @author Khizar Mohammed Amjed Mohamed
 */
public class MeshTerrainObjectTest
{
   private static final boolean SHOW_VISUALIZATION = false;
   private static final double SPHERE_RADIUS = 0.01;
   private static final double CUBE_DIMESNION = 0.01;

   private static SimulationConstructionSet scs = null;
   
   private static final int NUMBER_OF_ITERATIONS= 1000;
   private static final double MAXIMUM_ERROR = 0.02;
   private static final double BUFFER_VALUE = 0.05;
   
   private static final Random RANDOM = new Random();
   
   //LEGO BLOCK DIMENSIONS
   // Dimensions of A Zone; refer to drawing above
   private static double legoBlockAZoneMaximumZValue = 0.25 - BUFFER_VALUE;
   private static double legoBlockAZoneMinimumZValue = -0.25 + BUFFER_VALUE;
   private static double legoBlockAZoneMaximumYValue = 0.5 - BUFFER_VALUE;
   private static double legoBlockAZoneMinimumYValue = 0.0 + BUFFER_VALUE;
   
   // Dimensions of C Zone; refer to drawing above
   private static double legoBlockCZoneMaximumZValue = 0.0 - BUFFER_VALUE;
   private static double legoBlockCZoneMinimumZValue = -0.5 + BUFFER_VALUE;
   private static double legoBlockCZoneMaximumYValue = 0.0 - BUFFER_VALUE;
   private static double legoBlockCZoneMinimumYValue = -0.5 + BUFFER_VALUE;
   
   // Dimensions of comma Zone; refer to drawing above
   private static double legoBlockCommaZoneMaximumZValue = -0.25 - BUFFER_VALUE; 
   private static double legoBlockCommaZoneMinimumZValue = -5.0 + BUFFER_VALUE;
   
   private static double legoBlockCommaZoneMaximumYValue = 0.5 - BUFFER_VALUE ;
   private static double legoBlockCommaZoneMinimumYValue = 0.0 + BUFFER_VALUE;
   
   // Dimensions of dot Zone
   private static double legoBlockDotZoneMaximumZValue = 5.0 - BUFFER_VALUE; 
   private static double legoBlockDotZoneMinimumZValue = 0.0 + BUFFER_VALUE;
   private static double legoBlockDotZoneMaximumYValue = 0.0 - BUFFER_VALUE;
   private static double legoBlockDotZoneMinimumYValue = -0.5 + BUFFER_VALUE;
   
   // X Axis dimensions
   private static double legoBlockMaximumXValue = 0.5 - BUFFER_VALUE;
   private static double legoBlockMinimumXValue = 0.0 + BUFFER_VALUE;
   
   
   //HSHAPE BLOCK DIMENSIONS
   // A Zone Dimensions
   private static double hShapeAZoneMaximumZValue = 0.5 - BUFFER_VALUE;
   private static double hShapeAZoneMinimumZValue = -0.5 + BUFFER_VALUE;
   
   private static double hShapeAZoneMaximumXValue = -0.25 - BUFFER_VALUE;
   private static double hShapeAZoneMinimumXValue = -0.5 + BUFFER_VALUE;

   private static double hShapeAZoneMaximumYValue = 0.25 - BUFFER_VALUE;
   private static double hShapeAZoneMinimumYValue = -0.25 + BUFFER_VALUE;

   // Dimensions of B Zone
   private static double hShapeBZoneMaximumZValue = 0.25 - BUFFER_VALUE;
   private static double hShapeBZoneMinimumZValue = -0.25 + BUFFER_VALUE;

   private static double hShapeBZoneMaximumXValue = 0.25 - BUFFER_VALUE;
   private static double hShapeBZoneMinimumXValue = -0.25 + BUFFER_VALUE;

   private static double hShapeBZoneMaximumYValue = 0.25 - BUFFER_VALUE;
   private static double hShapeBZoneMinimumYValue = -0.25 + BUFFER_VALUE;
  
   // Dimensions of comma Zone
   private static double hShapeCommaZoneMaximumZValue = 0.5 - BUFFER_VALUE;
   private static double hShapeCommaZoneMinimumZValue = 0.25 + BUFFER_VALUE;
   
   // Dimensions of dot Zone
   private static double hShapeDotZoneMaximumZValue = -0.25 - BUFFER_VALUE;
   private static double hShapeDotZoneMinimumZValue = -0.5 + BUFFER_VALUE;

   @AfterEach
   public void tearDown()
   {
      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   @Test
   /**
    * This method is used for visual inspection of a MeshTerrainObject
    * <p>
    * Red is the point being tested - it will be a cube if heightAt is negative infinity, otherwise it
    * will be a sphere. Yellow ball is heightAt value if it exists
    * </p>
    * 
    * @author Khizar
    */
   public void testWithVisualization()
   {
      if (SHOW_VISUALIZATION)
      {

         RigidBodyTransform configuration = new RigidBodyTransform();
         Vector3D translation = new Vector3D(0.0, 0.0, 0.1);
         Quaternion orientation = new Quaternion(-Math.PI / 2.0, 0.0, 0.0);
         configuration.set(orientation, translation);
         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/walkway/walkway.obj", configuration);

         scs = new SimulationConstructionSet(new Robot("dummy"));
         scs.addStaticLinkGraphics(meshTerrainObject.getLinkGraphics());

         Graphics3DObject viz = new Graphics3DObject();

         // Adding testing points
         double xStart = -1.5;
         double xEnd = 1.5;
         double yStart = -0.75;
         double yEnd = 1.75;
         double increment = 0.3;

         for (double i = xStart; i < xEnd; i = i + increment)
         {
            for (double j = yStart; j < yEnd; j = j + increment)
            {
               addGraphicWithTestPoints(viz, meshTerrainObject, new Point3D(i, j, 2.0), YoAppearance.DarkSalmon());
            }
         }

         scs.addStaticLinkGraphics(viz);
         scs.setGroundVisible(false);
         scs.startOnAThread();
         ThreadTools.sleep(5000);

      }
   }

   @Test
   public void testintersectionWithVerticalLine()
   {
      String relativeFilePath = "models/legoBlock/legoBlock.obj";
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject(relativeFilePath);

      showVisualization(meshTerrainObject);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         // Randomly sample a point in the A zone
         double randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue) * RANDOM.nextDouble();
         double randomY = legoBlockAZoneMinimumYValue + (legoBlockAZoneMaximumYValue - legoBlockAZoneMinimumYValue) * RANDOM.nextDouble();

         IntersectionResult intersectionResult = meshTerrainObject.intersectionWithVerticalLine(randomX, randomY);
         Point3D actualHighestPoint = new Point3D(randomX, randomY, legoBlockAZoneMaximumZValue);
         Point3D actualLowestPoint = new Point3D(randomX, randomY, legoBlockAZoneMinimumZValue);

         Point3D highestPoint = intersectionResult.getHighestIntersection();
         Point3D lowestPoint = intersectionResult.getLowestIntersection();

         assertTrue(intersectionResult.isHighestPointValid());
         assertEquals(actualHighestPoint.differenceNormSquared(highestPoint), 0.0, MAXIMUM_ERROR);
         assertEquals(actualLowestPoint.differenceNormSquared(lowestPoint), 0.0, MAXIMUM_ERROR);

         // Randomly sample a point in the C zone
         randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue) * RANDOM.nextDouble();
         randomY = legoBlockCZoneMinimumYValue + (legoBlockCZoneMaximumYValue - legoBlockCZoneMinimumYValue) * RANDOM.nextDouble();

         intersectionResult = meshTerrainObject.intersectionWithVerticalLine(randomX, randomY);
         actualHighestPoint = new Point3D(randomX, randomY, legoBlockCZoneMaximumZValue);
         actualLowestPoint = new Point3D(randomX, randomY, legoBlockCZoneMinimumZValue);

         highestPoint = intersectionResult.getHighestIntersection();
         lowestPoint = intersectionResult.getLowestIntersection();

         String randomLocationAsString = "x:" + randomX +"   y:" + randomY;
         assertTrue(intersectionResult.isHighestPointValid());
         assertEquals(actualHighestPoint.differenceNormSquared(highestPoint), 0.0, MAXIMUM_ERROR, randomLocationAsString);
         assertEquals(actualLowestPoint.differenceNormSquared(lowestPoint), 0.0, MAXIMUM_ERROR, randomLocationAsString);

         // Randomly sample a point that does not have any vertical line intersection with the Mesh
         randomX = (0.6 + 5.0 * RANDOM.nextDouble()) * (RANDOM.nextBoolean() ? 1 : -1);
         randomY = (0.6 + 5.0 * RANDOM.nextDouble()) * (RANDOM.nextBoolean() ? 1 : -1);
         intersectionResult = meshTerrainObject.intersectionWithVerticalLine(randomX, randomY);
         assertFalse(intersectionResult.isHighestPointValid());
      }

   }

   @Test
   public void testHeightAt()
   {

      String relativeFilePath = "models/legoBlock/legoBlock.obj";
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject(relativeFilePath);

      showVisualization(meshTerrainObject);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         // Randomly sample a point inside the A zone
         double randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue) * RANDOM.nextDouble();
         double randomY = legoBlockAZoneMinimumYValue + (legoBlockAZoneMaximumYValue - legoBlockAZoneMinimumYValue) * RANDOM.nextDouble();
         double randomZ = legoBlockAZoneMinimumYValue + (legoBlockAZoneMaximumZValue - legoBlockAZoneMinimumZValue) * RANDOM.nextDouble();
         String randomLocationAsString = "x:" + randomX + " y:" + randomY + " z:" + randomZ;

         double heightAtValue = meshTerrainObject.heightAt(randomX, randomY, randomZ);
         double actualHeightAtValue = legoBlockAZoneMaximumZValue + BUFFER_VALUE;
         assertEquals(heightAtValue, actualHeightAtValue, MAXIMUM_ERROR, randomLocationAsString);

         // Randomly sample a point in the C zone
         randomY = legoBlockCZoneMinimumYValue + (legoBlockCZoneMaximumYValue - legoBlockCZoneMinimumYValue) * RANDOM.nextDouble();
         randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue) * RANDOM.nextDouble();
         randomZ = legoBlockCZoneMinimumYValue + (legoBlockCZoneMaximumZValue - legoBlockCZoneMinimumZValue) * RANDOM.nextDouble();
         randomLocationAsString = "x:" + randomX + " y:" + randomY + " z:" + randomZ;
         
         heightAtValue = meshTerrainObject.heightAt(randomX, randomY, randomZ);
         actualHeightAtValue = legoBlockCZoneMaximumZValue + BUFFER_VALUE;
         assertEquals(heightAtValue, actualHeightAtValue, MAXIMUM_ERROR, randomLocationAsString);

         // Randomly sample a point in the dot zone
         randomY = legoBlockDotZoneMinimumYValue + (legoBlockDotZoneMaximumYValue - legoBlockDotZoneMinimumYValue) * RANDOM.nextDouble();
         randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue) * RANDOM.nextDouble();
         randomZ = legoBlockDotZoneMinimumYValue + (legoBlockDotZoneMaximumZValue - legoBlockDotZoneMinimumZValue) * RANDOM.nextDouble();
         randomLocationAsString = "x:" + randomX + " y:" + randomY + " z:" + randomZ;

         heightAtValue = meshTerrainObject.heightAt(randomX, randomY, randomZ);
         actualHeightAtValue = legoBlockDotZoneMinimumZValue - BUFFER_VALUE;
         assertEquals(heightAtValue, actualHeightAtValue, MAXIMUM_ERROR, randomLocationAsString);

         // Randomly sample a point in the comma zone
         randomY = legoBlockCommaZoneMinimumYValue + (legoBlockCommaZoneMaximumYValue - legoBlockCommaZoneMinimumYValue) * RANDOM.nextDouble();
         randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue) * RANDOM.nextDouble();
         randomZ = legoBlockCommaZoneMinimumZValue + (legoBlockCommaZoneMaximumZValue - legoBlockCommaZoneMinimumZValue) * RANDOM.nextDouble();
         randomLocationAsString = "x:" + randomX + " y:" + randomY + " z:" + randomZ;

         heightAtValue = meshTerrainObject.heightAt(randomX, randomY, randomZ);
         actualHeightAtValue = Double.NEGATIVE_INFINITY;
         assertEquals(heightAtValue, actualHeightAtValue, MAXIMUM_ERROR, randomLocationAsString);
      }
   }
   public  void showVisualization(MeshTerrainObject meshTerrainObject)
   {
      if (SHOW_VISUALIZATION)
      {         
         scs = new SimulationConstructionSet(new Robot("dummy"));
         scs.addStaticLinkGraphics(meshTerrainObject.getLinkGraphics());
         scs.setGroundVisible(false);
         scs.startOnAThread();
         ThreadTools.sleep(5000);
      }
   }
   
   @Test
   public void testHeightAndNormalAt()
   {
      String relativeFilePath = "models/hShape/hShape.obj";
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject(relativeFilePath);

      showVisualization(meshTerrainObject);

      double randomX;
      double randomY;
      double randomZ;
      
      double actualHeightAtValue;
      double heightAtValue;
      
      Vector3D normalToPack = new Vector3D(0.0,0.0,0.0);
      Vector3D acualNormalToPack = new Vector3D(0.0, 0.0, 1.0);
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         // Randomly sample a point inside the a zone of the H Shape
         randomX = hShapeAZoneMinimumXValue + (hShapeAZoneMaximumXValue - hShapeAZoneMinimumXValue) * RANDOM.nextDouble();
         randomY = hShapeAZoneMinimumYValue + (hShapeAZoneMaximumYValue - hShapeAZoneMinimumYValue) * RANDOM.nextDouble();
         randomZ = hShapeAZoneMinimumZValue + (hShapeAZoneMaximumZValue - hShapeAZoneMinimumZValue) * RANDOM.nextDouble();
         
         actualHeightAtValue = hShapeAZoneMaximumZValue + BUFFER_VALUE;
         heightAtValue = meshTerrainObject.heightAndNormalAt(randomX, randomY, randomZ, normalToPack);
         assertEquals(actualHeightAtValue, heightAtValue, MAXIMUM_ERROR);
         assertEquals(acualNormalToPack.differenceNormSquared(normalToPack), 0.0, MAXIMUM_ERROR);
      }
      
   }

   @Test
   public void testCheckIfInside()
   {
      String relativeFilePath = "models/hShape/hShape.obj";
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject(relativeFilePath);

      showVisualization(meshTerrainObject);

      double randomX;
      double randomY;
      double randomZ;
      
      boolean isInside;
      Point3D intersectionToPack = new Point3D(0.0, 0.0, 0.0);
      Point3D actualIntersectionToPack;
      
      Vector3D normalToPack = new Vector3D(0.0,0.0,0.0);
      Vector3D acualNormalToPack = new Vector3D(0.0, 0.0, 1.0);
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         // Randomly sample a point inside the b zone
         randomX = hShapeBZoneMinimumXValue + (hShapeBZoneMaximumXValue - hShapeBZoneMinimumXValue) * RANDOM.nextDouble();
         randomY = hShapeBZoneMinimumYValue + (hShapeBZoneMaximumYValue - hShapeBZoneMinimumYValue) * RANDOM.nextDouble();
         randomZ = hShapeBZoneMinimumZValue + (hShapeBZoneMaximumZValue - hShapeBZoneMinimumZValue) * RANDOM.nextDouble();
         
         actualIntersectionToPack = new Point3D(randomX, randomY, hShapeBZoneMaximumZValue);
         isInside = meshTerrainObject.checkIfInside(randomX, randomY, randomZ, intersectionToPack, normalToPack);
         
         assertTrue(isInside);
         assertEquals(actualIntersectionToPack.differenceNormSquared(intersectionToPack), 0.0, MAXIMUM_ERROR);
         assertEquals(acualNormalToPack.differenceNormSquared(normalToPack), 0.0, MAXIMUM_ERROR);
         
         // Randomly sample a point inside the comma zone
         randomX = hShapeBZoneMinimumXValue + (hShapeBZoneMaximumXValue - hShapeBZoneMinimumXValue) * RANDOM.nextDouble();
         randomY = hShapeBZoneMinimumYValue + (hShapeBZoneMaximumYValue - hShapeBZoneMinimumYValue) * RANDOM.nextDouble();
         randomZ = hShapeCommaZoneMinimumZValue + (hShapeCommaZoneMaximumZValue - hShapeCommaZoneMinimumZValue) * RANDOM.nextDouble();
         
         actualIntersectionToPack = new Point3D(randomX, randomY, hShapeCommaZoneMinimumZValue);
         isInside = meshTerrainObject.checkIfInside(randomX, randomY, randomZ, intersectionToPack, normalToPack);
         
         assertFalse(isInside);
         assertEquals(actualIntersectionToPack.differenceNormSquared(intersectionToPack), 0.0, MAXIMUM_ERROR);
         assertEquals(acualNormalToPack.differenceNormSquared(normalToPack), 0.0, MAXIMUM_ERROR);

         // Randomly sample a point inside the dot zone
         randomX = hShapeBZoneMinimumXValue + (hShapeBZoneMaximumXValue - hShapeBZoneMinimumXValue) * RANDOM.nextDouble();
         randomY = hShapeBZoneMinimumYValue + (hShapeBZoneMaximumYValue - hShapeBZoneMinimumYValue) * RANDOM.nextDouble();
         randomZ = hShapeDotZoneMinimumZValue + (hShapeDotZoneMaximumZValue - hShapeDotZoneMinimumZValue) * RANDOM.nextDouble();
         
         actualIntersectionToPack = new Point3D(randomX, randomY, Double.NEGATIVE_INFINITY);
         isInside = meshTerrainObject.checkIfInside(randomX, randomY, randomZ, intersectionToPack, normalToPack);
         
         assertFalse(isInside);
         assertEquals(intersectionToPack.getZ(),Double.NEGATIVE_INFINITY);
      }
   }

   @Test
   public void testIsClose()
   {
      String legoBlockRelativeFilePath = "models/legoBlock/legoBlock.obj";
      MeshTerrainObject legoBlockMeshTerrainObject = new MeshTerrainObject(legoBlockRelativeFilePath);
      
      String hShapeRelativeFilePath = "models/hShape/hShape.obj";
      MeshTerrainObject hShapeMeshTerrainObject = new MeshTerrainObject(hShapeRelativeFilePath);

      // LegoBlock Bounding box limits
      double legoBlockMaximumXValue = 0.0;
      double legoBlockMinimumXValue = 0.5;
      
      double legoBlockMaximumYValue = 0.5;
      double legoBlockMinimumYValue = -0.5;
      
      double legoBlockMaximumZValue = 0.25;
      double legoBlockMinimumZValue = -0.5;
      
      // hShape Bounding box limits
      double hShapeMaximumYValue = 0.0;
      double hShapeMinimumYValue = -0.0;

      double hShapeMaximumXValue = 0.0;
      double hShapeMinimumXValue = -0.0;

      double hShapeMaximumZValue = 0.5;
      double hShapeMinimumZValue = -0.5;

      
      double randomX;
      double randomY;
      double randomZ;
      
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         // Randomly sample a point inside the bounding box for LegoBlocks
         randomX = legoBlockMinimumXValue + (legoBlockMaximumXValue - legoBlockMinimumXValue)*RANDOM.nextDouble(); 
         randomY = legoBlockMinimumYValue + (legoBlockMaximumYValue - legoBlockMinimumYValue)*RANDOM.nextDouble(); 
         randomZ = legoBlockMinimumZValue + (legoBlockMaximumZValue - legoBlockMinimumZValue)*RANDOM.nextDouble(); 
         assertTrue(legoBlockMeshTerrainObject.isClose(randomX, randomY, randomZ));
         
         // Randomly sample a point outside the bounding box for LegoBlocks
         randomX = (legoBlockMaximumXValue + RANDOM.nextDouble()*5.0)*(RANDOM.nextBoolean()? 1:-1);
         randomX = (legoBlockMaximumYValue + RANDOM.nextDouble()*5.0)*(RANDOM.nextBoolean()? 1:-1);
         randomX = (legoBlockMinimumZValue - RANDOM.nextDouble()*5.0)*(RANDOM.nextBoolean()? 1:-1);
         assertFalse(legoBlockMeshTerrainObject.isClose(randomX, randomY, randomZ));
         
         // Randomly sample a point inside the bounding box for hShape
         randomX = hShapeMinimumXValue + (hShapeMaximumXValue - hShapeMinimumXValue)*RANDOM.nextDouble(); 
         randomY = hShapeMinimumYValue + (hShapeMaximumYValue - hShapeMinimumYValue)*RANDOM.nextDouble(); 
         randomZ = hShapeMinimumZValue + (hShapeMaximumZValue - hShapeMinimumZValue)*RANDOM.nextDouble(); 
         assertTrue(hShapeMeshTerrainObject.isClose(randomX, randomY, randomZ));
         
         // Randomly sample a point outside the bounding box for hShape
         randomX = (hShapeMaximumXValue +RANDOM.nextDouble()*5.0)*(RANDOM.nextBoolean()?1:-1);
         randomX = (hShapeMaximumYValue +RANDOM.nextDouble()*5.0)*(RANDOM.nextBoolean()?1:-1);
         randomX = (hShapeMaximumZValue +RANDOM.nextDouble()*5.0)*(RANDOM.nextBoolean()?1:-1);
         assertFalse(legoBlockMeshTerrainObject.isClose(randomX, randomY, randomZ));
      }
   }

   /**
    * This method is used to visualize the heightat value of a MeshTerrainObject at a given point.
    * 
    * @author Khizar
    */
   /**
    * This method is used to draw arrows that align with the normal to the surface *
    * 
    * @author Khizar
    */
   public static void addArrowForNormal(Double xPoint, Double yPoint, Double heightAt, Vector3D normal)
   {

      YoRegistry robotsYoVariableRegistry = new YoRegistry("surfaceNormals");
      YoFramePoint3D planarRegionPointInWorld = new YoFramePoint3D("arrow", ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);
      Point3D translation = new Point3D(xPoint, yPoint, heightAt);
      planarRegionPointInWorld.set(translation);

      YoFrameVector3D surfaceNormal = new YoFrameVector3D("NormalVector", ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);
      surfaceNormal.set(normal);

      YoGraphicVector surfaceNormalGraphic = new YoGraphicVector("PlanarRegionSurfaceNormalGraphic",
                                                                 planarRegionPointInWorld,
                                                                 surfaceNormal,
                                                                 YoAppearance.Aqua());
      scs.addYoGraphic(surfaceNormalGraphic);

   }

   private static void addGraphicWithTestPoints(Graphics3DObject viz, MeshTerrainObject meshTerrainObject, Point3D point, AppearanceDefinition lineAppearance)
   {

      Double xPoint = point.getX();
      Double yPoint = point.getY();
      Double zPoint = point.getZ();

      Vector3D normalToPack = new Vector3D();
      normalToPack.set(0.0, 0.0, 0.0);
      double result = meshTerrainObject.heightAndNormalAt(xPoint, yPoint, zPoint, normalToPack);

      if (result != Double.NEGATIVE_INFINITY)
      {
         addArrowForNormal(xPoint, yPoint, result, normalToPack);

         viz.identity();
         viz.translate(xPoint, yPoint, zPoint);
         viz.addSphere(SPHERE_RADIUS, YoAppearance.Red());

         viz.identity();
         viz.translate(xPoint, yPoint, result);
         viz.addSphere(SPHERE_RADIUS, YoAppearance.Yellow());
      }
      else
      {

         viz.identity();
         viz.translate(xPoint, yPoint, zPoint);
         viz.addCube(CUBE_DIMESNION, CUBE_DIMESNION, CUBE_DIMESNION, YoAppearance.Red());
      }
      viz.identity();
      viz.translate(xPoint, yPoint, -500.0);
      viz.addCylinder(1000.0, 0.008, lineAppearance);

   }

}