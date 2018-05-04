package us.ihmc.simulationconstructionset.util.ground;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.commons.thread.ThreadTools;

public abstract class GroundProfileTest
{
   private boolean VISUALIZE = false;
   private boolean DEBUG = false;
   
// This magic number has been tuned based on how much height change beyond linear ground we should expect.
   private final double percentMaxHeightPerExcursionDistance = 0.1; 
   
   public abstract GroundProfile3D getGroundProfile();
   public abstract double getMaxPercentageOfAllowableValleyPoints();
   public abstract double getMaxPercentageOfAllowablePeakPoints();
   public abstract double getMaxPercentageOfAllowableDropOffs();

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testSurfaceNormalGridForSmoothTerrainUsingHeightMap()
   {
      Random random = new Random(1234L);
      
      GroundProfile3D groundProfile = getGroundProfile();
      HeightMapWithNormals heightMap = groundProfile.getHeightMapIfAvailable();
      if (heightMap == null) return;
      
      SimulationConstructionSet scs = null;
      BagOfBalls bagOfBalls = null;
      YoFramePoint3D surfaceNormalPointForViz = null;
      YoFrameVector3D surfaceNormalViz = null;
      if (VISUALIZE)
      {
         Robot robot = new Robot("Test");
         LinearStickSlipGroundContactModel groundModel = new LinearStickSlipGroundContactModel(robot, robot.getRobotsYoVariableRegistry());
         groundModel.setGroundProfile3D(groundProfile);

         robot.setGroundContactModel(groundModel);
         
         YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
         bagOfBalls = new BagOfBalls(robot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
         surfaceNormalPointForViz = new YoFramePoint3D("surfaceNormalPointForViz", ReferenceFrame.getWorldFrame(), robot.getRobotsYoVariableRegistry());
         surfaceNormalViz = new YoFrameVector3D("surfaceNormalVector", ReferenceFrame.getWorldFrame(), robot.getRobotsYoVariableRegistry());
         YoGraphicVector surfaceNormalGraphicVector = new YoGraphicVector("surfaceNormalViz", surfaceNormalPointForViz, surfaceNormalViz, YoAppearance.AliceBlue());
          
         yoGraphicsListRegistry.registerYoGraphic("Viz", surfaceNormalGraphicVector);
         
         scs = new SimulationConstructionSet(robot);
         scs.setGroundVisible(true);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

         scs.startOnAThread();
      }
      
      BoundingBox3D boundingBox = groundProfile.getBoundingBox();
      double xMin = boundingBox.getMinX();
      double yMin = boundingBox.getMinY();
      double xMax = boundingBox.getMaxX();
      double yMax = boundingBox.getMaxY();
      
      if (Double.isInfinite(xMin)) xMin = -10.0;
      if (Double.isInfinite(xMax)) xMax = 10.0;
      if (Double.isInfinite(yMin)) yMin = -10.0;
      if (Double.isInfinite(yMax)) yMax = 10.0;
      
      int numberOfPeakPoints = 0, numberOfValleyPoints = 0, numberOfDropOffChecks = 0;
      int numberOfTotalPoints = 0, numberOfTotalChecks = 0;
      
      int xSteps = 100;
      int ySteps = 100;
      
      double xStep = (xMax - xMin) / ((double) xSteps);
      double yStep = (yMax - yMin) / ((double) ySteps);
      
      for (int i = 2; i < xSteps-2; i++) // Minus 1 since we don't care about right up against the edges...
      {         
         double x = xMin + ((double) i) * xStep;
         for (int j=2; j < ySteps-2; j++)
         {
            double y = yMin + ((double) j) * yStep;
            Vector3D surfaceNormal = new Vector3D();
            double height = heightMap.heightAndNormalAt(x, y, 0.0, surfaceNormal);
            assertEquals(1.0, surfaceNormal.length(), 1e-7);

            Point3D queryPoint = new Point3D(x, y, height);
            numberOfTotalPoints++;
            
            Point3D queryPointALittleInside = new Point3D(queryPoint);
            queryPointALittleInside.scaleAdd(-0.002, surfaceNormal, queryPointALittleInside);
            
            Point3D queryPointALittleOutside = new Point3D(queryPoint);
            queryPointALittleOutside.scaleAdd(0.002, surfaceNormal, queryPointALittleOutside);
            
//            System.out.println("queryPoint = " + queryPoint);
//            System.out.println("queryPointALittleInside = " + queryPointALittleInside);
            Vector3D surfaceNormalCheck= new Vector3D();
            Point3D intersectionCheck = new Point3D();
            boolean insideShouldBeTrue = groundProfile.checkIfInside(queryPointALittleInside.getX(), queryPointALittleInside.getY(), queryPointALittleInside.getZ(), intersectionCheck, surfaceNormalCheck);
            
            // If surface normals are not close when looking a little inside, then might likely have a peak
            assertEquals(1.0, surfaceNormal.length(), 1e-7);
            
            boolean surfaceNormalsAreClose = surfaceNormalCheck.dot(surfaceNormal) > 1.0 - 0.001;
            boolean belowAPeak = !surfaceNormalsAreClose;
            if (belowAPeak) numberOfPeakPoints++; //assertTrue(surfaceNormalsAreClose);
            else 
            {
               assertTrue(insideShouldBeTrue);
               assertTrue(groundProfile.isClose(queryPointALittleInside.getX(), queryPointALittleInside.getY(), queryPointALittleInside.getZ()));
            }

            EuclidCoreTestTools.assertTuple3DEquals(intersectionCheck, queryPoint, 0.002);
            
            boolean outsideShouldBeFalse = groundProfile.checkIfInside(queryPointALittleOutside.getX(), queryPointALittleOutside.getY(), queryPointALittleOutside.getZ(), intersectionCheck, surfaceNormalCheck);
            
            // If surface normals are not close when looking a little outside, then might likely have a valley
            assertEquals(1.0, surfaceNormal.length(), 1e-7);
            surfaceNormalsAreClose = surfaceNormalCheck.dot(surfaceNormal) > 1.0 - 0.001;
            boolean aboveAValley = !surfaceNormalsAreClose;

            if (aboveAValley) numberOfValleyPoints++; //assertTrue(surfaceNormalsAreClose);
            else assertFalse(outsideShouldBeFalse);

            EuclidCoreTestTools.assertTuple3DEquals(intersectionCheck, queryPoint, 0.002);

            if (VISUALIZE)
            {
               // Draw a normal vector. Make it long if on a peak and short if in a valley.
               Vector3D normalToDraw = new Vector3D(surfaceNormal);
               if (aboveAValley) normalToDraw.scale(0.5);
               else if (belowAPeak) normalToDraw.scale(2.0);
               
               surfaceNormalPointForViz.set(queryPoint);
               surfaceNormalViz.set(normalToDraw);
               
               AppearanceDefinition appearance = YoAppearance.AliceBlue();

               bagOfBalls.setBallLoop(new FramePoint3D(ReferenceFrame.getWorldFrame(), queryPoint), appearance);
               scs.setTime(scs.getTime() + 0.001);
               scs.tickAndUpdate();
            }
            
            // Can only be either above a valley or below a peak...
            assertFalse(aboveAValley && belowAPeak); 

            if (!aboveAValley && !belowAPeak)
            {
               // Move a little and see that mostly a flat plane, when not in a valley or on a peak.
               Vector3D alongDirectionOne = new Vector3D();
               Vector3D alongDirectionTwo = new Vector3D();

               alongDirectionOne.set(1.0, 0.0, 0.0);
               alongDirectionTwo.cross(surfaceNormal, alongDirectionOne);
               alongDirectionTwo.normalize();
               alongDirectionOne.cross(alongDirectionTwo, surfaceNormal);

               int numberOfChecks = 10;
               double excursionDistance = 0.001; 
               double maxTolerableHeightDifferenceFromLinearModel = percentMaxHeightPerExcursionDistance * excursionDistance;

               for (int k=0; k<numberOfChecks; k++)
               {
                  numberOfTotalChecks++;
                  
                  Vector2D excursionVector2d = RandomGeometry.nextVector2D(random, excursionDistance);
                  Vector3D excursionVector = new Vector3D(alongDirectionOne);
                  excursionVector.scale(excursionVector2d.getX());
                  excursionVector.scaleAdd(excursionVector2d.getY(), alongDirectionTwo, excursionVector);

                  queryPoint.set(x, y, height);
                  queryPoint.add(excursionVector);

                  double heightAtQueryPoint = heightMap.heightAt(queryPoint.getX(), queryPoint.getY(), queryPoint.getZ());
                  double heightDifferenceFromLinearModel = queryPoint.getZ() - heightAtQueryPoint;

                  boolean hitADropOff = Math.abs(heightDifferenceFromLinearModel) > maxTolerableHeightDifferenceFromLinearModel;

                  if (hitADropOff && DEBUG)
                  {
                     System.out.println("(x, y, z) = (" + x + ", " + y + ", " + height + ")");
                     System.out.println("surfaceNormal = " + surfaceNormal);
                     System.out.println("alongDirectionOne = " + alongDirectionOne);
                     System.out.println("alongDirectionTwo = " + alongDirectionTwo);
                     System.out.println("excursionVector = " + excursionVector);
                     System.out.println("queryPoint = " + queryPoint);
                     System.out.println("heightAtQueryPoint = " + heightAtQueryPoint);
                     System.out.println("heightDifferenceFromLinearModel = " + heightDifferenceFromLinearModel);

                     System.out.println();
                  }

                  if (hitADropOff) numberOfDropOffChecks++;
               }
            }
         }
      }
      
      double percentPeakPoints = numberOfPeakPoints/((double) numberOfTotalPoints);
      double percentValleyPoints = numberOfValleyPoints/((double) numberOfTotalPoints);
      
      double percentDropOffChecks = numberOfDropOffChecks / ((double) numberOfTotalChecks);
      
      boolean tooManyPeakPoints = percentPeakPoints > getMaxPercentageOfAllowablePeakPoints();
      boolean tooManyValleyPoints = percentValleyPoints > getMaxPercentageOfAllowableValleyPoints();
      
      boolean tooManyDropOffChecks = percentDropOffChecks > getMaxPercentageOfAllowableDropOffs();

      if (tooManyPeakPoints || tooManyValleyPoints || tooManyDropOffChecks)
      {
         System.out.println("numberOfPeakPoints = " + numberOfPeakPoints);
         System.out.println("numberOfValleyPoints = " + numberOfValleyPoints);
         System.out.println("numberOfTotalPoints = " + numberOfTotalPoints);
         System.out.println("percentPeakPoints = " + percentPeakPoints);
         System.out.println("percentValleyPoints = " + percentValleyPoints);
         
         System.out.println("numberOfDropOffChecks = " + numberOfDropOffChecks);
         System.out.println("numberOfTotalChecks = " + numberOfTotalChecks);
         System.out.println("percentDropOffChecks = " + percentDropOffChecks);
      }
      
      assertFalse(tooManyPeakPoints);
      assertFalse(tooManyValleyPoints);
      assertFalse(tooManyDropOffChecks);
      
      if (VISUALIZE)
      {
         ThreadTools.sleepForever();
      }
   }

}
