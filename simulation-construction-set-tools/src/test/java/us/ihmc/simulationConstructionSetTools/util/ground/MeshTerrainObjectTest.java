package us.ihmc.simulationConstructionSetTools.util.ground;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MeshTerrainObjectTest
{
   private static final boolean SHOW_VISUALIZATION = true;
   private static SimulationConstructionSet scs = null;

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
         configuration.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, -Math.PI / 2.0));

         //         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/SmallWalkway/Walkway.obj", configuration);
         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/Walkway/Walkway.obj", configuration);
         //         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/Valley/Valley.obj", configuration);

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
         ThreadTools.sleepForever();

      }
   }

   @Test
   /**
    * This method is used to test the method heightAt() of a MeshTerrainObject
    * <p>
    * Red is the point being tested - it will be a cube if heightAt is negative infinity, otherwise it
    * will be a sphere. Yellow ball is heightAt value if it exists.
    * </p>
    * 
    * @author Khizar
    */
   public void testHeightAt()
   {

      // Testing using a Stool with bounding box dimensions as X,Y,Z = 0.375, 0.375,0.575
      // Stool dimensions can be found here "https://cad.onshape.com/documents/ed770d6e6aa0b5ca25885953/w/d5578c1a76bb6f06ae50c7b9/e/ebb25271027fb47ebaec1c9c"

      String relativeFilePath = "models/Stool/Stool.obj";
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject(relativeFilePath);

      Double heightAt;
      Double maxError = 0.001;
      // Point A is above the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will intersect the stool
      Point3D pointA = new Point3D(0.25, 0.25, 0.6);
      heightAt = meshTerrainObject.heightAt(pointA.getX(), pointA.getY(), pointA.getZ());
      assertEquals(heightAt, 0.575, maxError);

      // Point B is under the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still intersect the stool
      Point3D pointB = new Point3D(0.18, 0.18, 0.25);
      heightAt = meshTerrainObject.heightAt(pointB.getX(), pointB.getY(), pointB.getZ());
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);

      // Point C is inside the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still intersect the stool
      Point3D pointC = new Point3D(0.18, 0.18, 0.5);
      heightAt = meshTerrainObject.heightAt(pointC.getX(), pointC.getY(), pointC.getZ());
      assertEquals(heightAt, 0.575, maxError);

      // Point D is above the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still NOT INTERSECT the stool
      Point3D pointD = new Point3D(0.5, 0.5, 0.75);
      heightAt = meshTerrainObject.heightAt(pointD.getX(), pointD.getY(), pointD.getZ());
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);

      // Point E is below the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still NOT INTERSECT the stool
      Point3D pointE = new Point3D(-0.25, -0.25, -0.25);
      heightAt = meshTerrainObject.heightAt(pointE.getX(), pointE.getY(), pointE.getZ());
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);

      // Point F is on the top surface of the stool. If an infinite line is drawn and parallel to the Z axis of this point, the line will INTERSECT the stool
      Point3D pointF = new Point3D(0.18, 0.18, 0.575);
      heightAt = meshTerrainObject.heightAt(pointF.getX(), pointF.getY(), pointF.getZ());
      assertEquals(heightAt, 0.575, maxError);

      Graphics3DObject viz = new Graphics3DObject();
      scs = new SimulationConstructionSet(new Robot("dummy"));
      scs.addStaticLinkGraphics(meshTerrainObject.getLinkGraphics());

      scs.setGroundVisible(false);

      addGraphicWithTestPoints(viz, meshTerrainObject, pointA, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointB, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointC, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointD, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointE, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointF, YoAppearance.DarkSalmon());

      // Red is the point being tested - it will be a cube if heightAt is negative
      // infinity, otherwise it will be a sphere
      // Yellow ball is heightAt value if it exists

      scs.addStaticLinkGraphics(viz);
      scs.startOnAThread();
      ThreadTools.sleepForever();

   }

   @Test
   public void testCheckIfInside()
   {
      // METHOD NOTR YET IMPLEMENTED FOR MESH TERRAIN OBJECTS
   }

   @Test
   public void testHeightAndNormalAt()
   {
      // METHOD NOTR YET IMPLEMENTED FOR MESH TERRAIN OBJECTS
   }

   @Test
   public void testIsClose()
   {
      // METHOD NOTR YET IMPLEMENTED FOR MESH TERRAIN OBJECTS

   }

   /**
    * This method is used to visualize the heightat value of a MeshTerrainObject at a given point.
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
   /**
    * This method is used to draw arrows that align with the normal to the surface    * 
    * @author Khizar
    */
   private static void addGraphicWithTestPoints(Graphics3DObject viz,
                                                MeshTerrainObject convexPolytopeTerrainObject,
                                                Point3D point,
                                                AppearanceDefinition lineAppearance)
   {

      Double xPoint = point.getX();
      Double yPoint = point.getY();
      Double zPoint = point.getZ();

      Vector3D normalToPack = new Vector3D();
      normalToPack.set(0.0, 0.0, 0.0);
      double result = convexPolytopeTerrainObject.heightAndNormalAt(xPoint, yPoint, zPoint, normalToPack);

      if (result != Double.NEGATIVE_INFINITY)
      {
         addArrowForNormal(xPoint, yPoint, result, normalToPack);

         viz.identity();
         viz.translate(xPoint, yPoint, zPoint);
         viz.addSphere(0.04, YoAppearance.Red());

         viz.identity();
         viz.translate(xPoint, yPoint, result);
         viz.addSphere(0.04, YoAppearance.Yellow());
      }
      else
      {

         viz.identity();
         viz.translate(xPoint, yPoint, zPoint);
         viz.addCube(0.08, 0.08, 0.08, YoAppearance.Red());
      }
      viz.identity();
      viz.translate(xPoint, yPoint, -500.0);
      viz.addCylinder(1000.0, 0.008, lineAppearance);

   }

}