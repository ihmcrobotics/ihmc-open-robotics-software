package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.vecmath.Point2d;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPolygon;

public class SmartCMPProjectorTest
{
   private static final boolean SHOW_SCS = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testSimpleProjections()
   {
      YoVariableRegistry registry;
      SimulationConstructionSet scs;

      if (SHOW_SCS)
      {
         scs = new SimulationConstructionSet(new Robot("null"));
         registry = scs.getRootRegistry();
      }
      else
      {
         scs = null;
         registry = new YoVariableRegistry("Test");
      }

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(registry, dynamicGraphicObjectsListRegistry);
      double cmpEdgeProjectionInside = 0.05;
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      double minICPToCMPProjection = 0.02;
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      if (SHOW_SCS)
      {
         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         createOverheadPlotter(dynamicGraphicObjectsListRegistry, scs, true);
         scs.startOnAThread();
      }

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Test all in same place
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.5, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge.

      desiredCMP = new FramePoint2d(worldFrame, 1.0, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);


      // Test simple projection away from edge.
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge.
      desiredCMP = new FramePoint2d(worldFrame, 1.02, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge.
      desiredCMP = new FramePoint2d(worldFrame, 1.9, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, 0.94, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.96, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, 0.94, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 1.5, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.96, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, 0.94, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.95, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.01, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.01, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test just projecting to the ICP.
      capturePoint = new FramePoint2d(worldFrame, 1.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 1.5, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test projecting to the edge.
      capturePoint = new FramePoint2d(worldFrame, -2.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, -0.1, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
      
      // Test with narrow foot
      pointList = new double[][]
      {
         {0.0, 0.0}, {0.1, 0.0}, {0.1, 1.0}, {0.0, 1.0}
      };
      supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      capturePoint = new FramePoint2d(worldFrame, 0.05, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.05, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.05, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test with narrow foot
      capturePoint = new FramePoint2d(worldFrame, 0.01, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.02, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.02, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test with narrow foot
      capturePoint = new FramePoint2d(worldFrame, 0.02, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.01, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.01, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);


      if (SHOW_SCS)
      {
         ThreadTools.sleepForever();
      }
   }


   @Test
   public void testTroublesomeOne()
   {      
      double[][] pointList = new double[][]
      {
         {-4.979747892521815, 0.5541117019274466}, {-0.42026607108138236, 1.9379654867165463}, {6.119471235760535, -3.9598753931171444},
         {2.0773903942158043, -6.074548734056259}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0533200632454793, 1.6302464329466666);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -0.06917411248032934, 7.395654149367431);

      double cmpEdgeProjectionInside = 0.056828294292739036;
      double minICPToCMPProjection = 0.14855380619424705;
      
      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(null, null);
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);
     
      FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);
      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, projectedCMP);
      
      assertSolutionIsReasonable(cmpEdgeProjectionInside, minICPToCMPProjection, supportPolygon,
          capturePoint, desiredCMP, projectedCMP);
   }
   
   @Test
   public void testTroublesomeTwo()
   {
      double[][] pointList = new double[][]
            {
            {-8.200433598264752, 4.736778327900604},
            {-7.473324755152609, 8.139207651739621},
            {4.267220732947754, 9.671770258736647},
            {4.341938271519323, -4.367767456386648},
            {2.389406061561967, -7.47468823044664},
            {-3.2041110411643086, -5.730732619977008}
            };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -7.601990600688118, -4.41238077781491);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -5.030652009330159, -2.246171870476954);

      double cmpEdgeProjectionInside = 0.021954883046042744;
      double minICPToCMPProjection = 0.09194115707136612;

      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(null, null);
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);
      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, projectedCMP);
      
      assertSolutionIsReasonable(cmpEdgeProjectionInside, minICPToCMPProjection, supportPolygon,
            capturePoint, desiredCMP, projectedCMP);
   }
   
   @Test
   public void testTroublesomeThree()
   {
      double[][] pointList = new double[][]
            {
            {-8.200433598264752, 4.736778327900604},
            {-7.473324755152609, 8.139207651739621},
            {4.267220732947754, 9.671770258736647},
            {4.341938271519323, -4.367767456386648},
            {2.389406061561967, -7.47468823044664},
            {-3.2041110411643086, -5.730732619977008}
            };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -7.601990600688118, -4.41238077781491);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -5.030652009330159, -2.246171870476954);

      double cmpEdgeProjectionInside = 0.021954883046042744;
      double minICPToCMPProjection = 0.09194115707136612;

      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(null, null);
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);
      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, projectedCMP);      
      assertSolutionIsReasonable(cmpEdgeProjectionInside, minICPToCMPProjection, supportPolygon,
            capturePoint, desiredCMP, projectedCMP);
   }
   
   @Test
   public void testTroublesomeFour()
   {
      double[][] pointList = new double[][]
            {
            {-9.866922926359909, 3.620889108752019},
            {-6.106666508735787, 9.488161702372114},
            {7.281277900378459, 2.4430532928556126},
            {8.761409532045942, -1.1373581188191206},
            {7.705064492576788, -5.27367712948074},
            {6.242601307942824, -7.091872404831583},
            {5.513393774314316, -7.496454748096967},
            {-1.6921535955371763, -2.4479235906163606}
            };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -5.310400543247988, 0.24850573514182805);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -6.217613490956406, 0.7306813725318086);

      double cmpEdgeProjectionInside = 0.0022412333001730405;
      double minICPToCMPProjection = 0.1592972381774514;

      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(null, null);
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);
      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, projectedCMP);
      
      assertSolutionIsReasonable(cmpEdgeProjectionInside, minICPToCMPProjection, supportPolygon,
            capturePoint, desiredCMP, projectedCMP);
   }

   @Ignore
   @Test
   public void testTroublesomeFive()
   {
      Vizzy viz = new Vizzy();

      double[][] pointList = new double[][]
            {
            {-6.6364442778312505, 5.081037775538617},
            {-5.809348495016624, 5.2896472244805715},
            {2.795403317979172, 5.62762983725845},
            {-0.47916782691148363, 3.346181735352964},
            {-6.593198911501661, 0.0730878970354496}
            };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      viz.addConvexPolygon(supportPolygon);

      // Test simple projection away from edge, but not too far.
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 6.203278757526469, -4.079058120870805);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 2.573986759898151, 5.590979195575493);

      double cmpEdgeProjectionInside = 0.1407917153012075;
           double minICPToCMPProjection = 0.04701036473699262;

      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(viz.getYoVariableRegistry(), viz.getDynamicGraphicObjectsListRegistry());
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      viz.start();

      FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);

      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, projectedCMP);
      tickAndUpdate(viz.getSimulationConstructionSet());
      
      assertSolutionIsReasonable(cmpEdgeProjectionInside, minICPToCMPProjection, supportPolygon,
            capturePoint, desiredCMP, projectedCMP);
      
      ThreadTools.sleepForever();
   }
   
   @Test
   public void testSimpleProjectionAwayFromEdgeButNotTooFar()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      // Test simple projection away from edge, but not too far.
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }
   
   @Test
   public void testSimpleProjectionICPOutside()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.5, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }
   
   @Test
   public void testSimpleProjectionICPOutsideTwo()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 1.01, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }
   
   @Test
   public void testSimpleProjectionICPOutsideThree()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 0.01, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.01, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }
   
   @Test
   public void testSimpleProjectionBackOntoTheEdge()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -0.1, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }
   
   
   @Test
   public void testICPInsideCMPOutside()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -0.1, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.05, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }
   
   @Test
   public void testICPInsideCloseCMPOutside()
   {
      SmartCMPProjector smartCMPProjector = createSmartCMPProjector();
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.01, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -0.01, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 0.5);
      checkOne(null, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }

   @Test
   public void testRandomPoints()
   {
      Random random = new Random(1775L);

      //TODO: If this is one million, we still have some problems. Fix the remaining outliers.
      int numberOfTests = 10000;

      for (int testNumber = 0; testNumber < numberOfTests; testNumber++)
      {
         SmartCMPProjector smartCMPProjector = new SmartCMPProjector(null, null);
         double cmpEdgeProjectionInside = RandomTools.generateRandomDouble(random, 0.001, 0.2);
         smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
         double minICPToCMPProjection = RandomTools.generateRandomDouble(random, 0.001, 0.2);
         smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

         int numberOfPoints = RandomTools.generateRandomInt(random, 4, 20);
         double[][] pointList = new double[numberOfPoints][2];
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2d point = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
            pointList[i][0] = point.getX();
            pointList[i][1] = point.getY();
         }

         FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

         // Test simple projection away from edge, but not too far.
         FramePoint2d capturePoint = new FramePoint2d(worldFrame, RandomTools.generateRandomPoint2d(random, 10.0, 10.0));
         FramePoint2d desiredCMP = new FramePoint2d(worldFrame, RandomTools.generateRandomPoint2d(random, 10.0, 10.0));
         FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);

         smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, projectedCMP);

         assertSolutionIsReasonable(cmpEdgeProjectionInside, minICPToCMPProjection, supportPolygon, capturePoint, desiredCMP, projectedCMP);
      }
   }


   private void assertSolutionIsReasonable(double cmpEdgeProjectionInside, double minICPToCMPProjection, FrameConvexPolygon2d supportPolygon,
         FramePoint2d capturePoint, FramePoint2d desiredCMP, FramePoint2d projectedCMP)
   {
      boolean originalCMPIsInside = supportPolygon.isPointInside(desiredCMP, 1e-5);
      boolean capturePointIsInside = supportPolygon.isPointInside(capturePoint, 1e-5);
      boolean projectedCMPIsInside = supportPolygon.isPointInside(projectedCMP, 1e-5);

      if (originalCMPIsInside || capturePointIsInside)
      {
         if (!projectedCMPIsInside)
         {
     
               System.err.println("\n\nsupportPolygon = " + supportPolygon);
               System.err.println("cmpEdgeProjectionInside = " + cmpEdgeProjectionInside);
               System.err.println("minICPToCMPProjection = " + minICPToCMPProjection);
               System.err.println("cmpEdgeProjectionInside = " + cmpEdgeProjectionInside);
               System.err.println("capturePoint = " + capturePoint);
               System.err.println("desiredCMP = " + desiredCMP);
               System.err.println("projectedCMP = " + projectedCMP);

               System.err.println("originalCMPIsInside = " + originalCMPIsInside);
               System.err.println("capturePointIsInside = " + capturePointIsInside);
               System.err.println("projectedCMPIsInside = " + projectedCMPIsInside);
               
//               System.err.println("distanceFromProjectedToCapturePoint = " + distanceFromProjectedToCapturePoint);
//               System.err.println("distanceFromOriginalToCapturePoint = " + distanceFromOriginalToCapturePoint);

         }
         
       assertTrue(projectedCMPIsInside);
      }

      // Make sure projected is either at capture point, or on line between desired CMP and capture Point.
      double distanceFromProjectedToCapturePoint = projectedCMP.distance(capturePoint);
      double distanceFromOriginalToCapturePoint = desiredCMP.distance(capturePoint);
      
      boolean specialCaseWhereCMPCanBecomeFurtherFromCapturePoint = false;
      
      if (!capturePointIsInside & !originalCMPIsInside)
      {
         FrameLineSegment2d lineSegment = new FrameLineSegment2d(capturePoint, desiredCMP);
         FramePoint2d[] intersectionWith = supportPolygon.intersectionWith(lineSegment);
         if ((intersectionWith == null) || (intersectionWith.length == 0))
         {
            specialCaseWhereCMPCanBecomeFurtherFromCapturePoint = true;
         }
      }
      
      if (!specialCaseWhereCMPCanBecomeFurtherFromCapturePoint)
      {
         assertTrue(distanceFromProjectedToCapturePoint <= distanceFromOriginalToCapturePoint);
      }

      if (distanceFromProjectedToCapturePoint < 1e-7)
      {
         FrameVector2d originalToCapturePoint = new FrameVector2d(capturePoint);
         originalToCapturePoint.sub(desiredCMP);

         FrameVector2d projectedToCapturePoint = new FrameVector2d(capturePoint);
         originalToCapturePoint.sub(projectedCMP);

         assertTrue(originalToCapturePoint.dot(projectedToCapturePoint) >= 0.0);
         double cross = originalToCapturePoint.cross(projectedToCapturePoint);
         assertTrue(Math.abs(cross) < 1e-7);
      }
   }

   private SmartCMPProjector createSmartCMPProjector()
   {
      double cmpEdgeProjectionInside = 0.05;
      double minICPToCMPProjection = 0.02;

      return createSmartCMPProjector(cmpEdgeProjectionInside, minICPToCMPProjection);
   }

   private SmartCMPProjector createSmartCMPProjector(double cmpEdgeProjectionInside, double minICPToCMPProjection)
   {
      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(null, null);
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      return smartCMPProjector;
   }


   private void checkOne(SimulationConstructionSet scs, SmartCMPProjector smartCMPProjector, FrameConvexPolygon2d supportPolygon, FramePoint2d capturePoint,
                         FramePoint2d desiredCMP, FramePoint2d expectedCMPProjection)
   {
      FramePoint2d desiredCMPProjection = new FramePoint2d(desiredCMP);

      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, desiredCMPProjection);
      tickAndUpdate(scs);
      assertTrue(expectedCMPProjection.epsilonEquals(desiredCMPProjection, 1e-7));
   }

   private void tickAndUpdate(SimulationConstructionSet scs)
   {
      if (scs != null)
      {
         scs.tickAndUpdate();
      }
   }

   private static void createOverheadPlotter(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, SimulationConstructionSet scs,
           boolean showOverheadView)
   {
      SimulationOverheadPlotter plotter = new SimulationOverheadPlotter();
      plotter.setDrawHistory(false);
      plotter.setXVariableToTrack(null);
      plotter.setYVariableToTrack(null);

      scs.attachPlaybackListener(plotter);
      JPanel plotterPanel = plotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(plotterPanel, plotterName);
      JPanel plotterKeyJPanel = plotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);

      scs.addExtraJpanel(scrollPane, "Plotter Legend");

      dynamicGraphicObjectsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());

      if (showOverheadView)
         scs.getStandardSimulationGUI().selectPanel(plotterName);
   }

   private class Vizzy
   {
      private final YoVariableRegistry registry;
      private final SimulationConstructionSet scs;
      private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;

      public Vizzy()
      {
         scs = new SimulationConstructionSet(new Robot("null"));
         registry = scs.getRootRegistry();

         dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      }

      public void addConvexPolygon(FrameConvexPolygon2d supportPolygon)
      {
         DynamicGraphicPolygon dynamicGraphicPolygon = new DynamicGraphicPolygon("polygon", supportPolygon.getConvexPolygon2dCopy(), "polygon", "", registry, 1.0, YoAppearance.Red());
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Polygon", dynamicGraphicPolygon);
         
//         dynamicGraphicObjectsListRegistry.registerArtifact("Polygon", dynamicGraphicPolygon.createArtifact());
         
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public DynamicGraphicObjectsListRegistry getDynamicGraphicObjectsListRegistry()
      {
         return dynamicGraphicObjectsListRegistry;
      }

      public SimulationConstructionSet getSimulationConstructionSet()
      {
         return scs;
      }

      public void start()
      {
         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         createOverheadPlotter(dynamicGraphicObjectsListRegistry, scs, true);
         scs.startOnAThread();
      }



   }

}
