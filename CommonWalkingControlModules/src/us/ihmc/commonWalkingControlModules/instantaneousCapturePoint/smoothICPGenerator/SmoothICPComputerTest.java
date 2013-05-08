package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.test.JUnitTools;

public class SmoothICPComputerTest
{
   @Test
   public void testTypicalFourStepExample()
   {
      boolean visualize = true;

      int maxNumberOfConsideredFootsteps = 4;
      double doubleSupportFirstStepFraction = 0.45;
      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction , maxNumberOfConsideredFootsteps);

      ArrayList<Point3d> footLocations = new ArrayList<Point3d>();
      double height = 1.2;
      
      footLocations.add(new Point3d(0.0, 0.0, height));
      footLocations.add(new Point3d(0.21, 0.22, height + 0.01));
      footLocations.add(new Point3d(0.02, 0.42, height - 0.05));
      footLocations.add(new Point3d(0.2, 0.63, height));

      PointAndLinePlotter pointAndLinePlotter = null;
      SimulationConstructionSet scs = null;
      YoVariableRegistry registry = null;
      if (visualize)
      {
         Robot robot = new Robot(getClass().getSimpleName());
         scs = new SimulationConstructionSet(robot);
         registry = robot.getRobotsYoVariableRegistry();
         pointAndLinePlotter = new PointAndLinePlotter(registry);
         pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

         pointAndLinePlotter.plotPoint3ds("footLocations", footLocations, YoAppearance.Black(), 0.01);
      }
      
      double singleSupportDuration = 0.5;
      double doubleSupportDuration = 0.2;

      double initialTime = 1.1;
      double omega0 = 3.0;
      
      smoothICPComputer.initializeSingleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime);
      
      if (visualize)
      {
         Point3d[] icpCornerPoints = smoothICPComputer.getICPCornerPoints();
         pointAndLinePlotter.plotPoint3ds("icpCornerPoints", icpCornerPoints, YoAppearance.Green(), 0.01);
      }
      
      Point3d initialICPPosition = new Point3d();
      Vector3d initialICPVelocity = new Vector3d();
      smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, omega0, initialTime);

      Point3d transferFromFoot = footLocations.get(0);

      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();
      
      simulateForwardAndCheckSingleSupport(smoothICPComputer, singleSupportDuration, initialTime, omega0, initialICPPosition,
            transferFromFoot, icpPosition, icpVelocity);
            
      initialTime = initialTime + singleSupportDuration;
      smoothICPComputer.initializeDoubleSupport(footLocations, singleSupportDuration, doubleSupportDuration, omega0, initialTime);
      smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, omega0, initialTime);
      
      
      
      
      JUnitTools.assertTuple3dEquals(icpPosition, initialICPPosition, 1e-4);
      JUnitTools.assertTuple3dEquals(icpVelocity, initialICPVelocity, 1e-4);
      
      simulateForwardAndCheckDoubleSupport(smoothICPComputer, doubleSupportDuration, initialTime, omega0, initialICPPosition,
            transferFromFoot, icpPosition, icpVelocity);
      
      initialTime = initialTime + doubleSupportDuration;
      footLocations.remove(0);
      footLocations.add(new Point3d(0.0, 4.0, 0.0));
      transferFromFoot = footLocations.get(0);
//
//      smoothICPComputer.initializeSingleSupport(footLocations, singleSupportDuration, doubleSupportDuration, initialTime, omega0);
//      smoothICPComputer.getICPPositionAndVelocity(initialICPPosition, initialICPVelocity, omega0, initialTime);
//      
//      simulateForwardAndCheckSingleSupport(smoothICPComputer, singleSupportDuration, initialTime, omega0, initialICPPosition,
//            transferFromFoot, icpPosition, icpVelocity); 
      
      
      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }

   private void simulateForwardAndCheckSingleSupport(SmoothICPComputer smoothICPComputer, double singleSupportDuration, double initialTime,
         double omega0, 
         Point3d initialICPPosition, Point3d transferFromFoot, Point3d icpPosition, Vector3d icpVelocity)
   {
      double deltaT = 0.001;
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + singleSupportDuration; time = time + deltaT)
      {
         smoothICPComputer.getICPPositionAndVelocity(icpPosition, icpVelocity, omega0, time);
         assertICPPointsAreAlongLineIncludingTransferFromFoot(transferFromFoot, initialICPPosition, icpPosition);

         Vector3d approximateVelocity = new Vector3d(icpPosition);
         approximateVelocity.sub(previousICPPosition);
         approximateVelocity.scale(1.0 / deltaT);

         JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocity, 1e-3);
         previousICPPosition.set(icpPosition);
      }
   }
   
   private void simulateForwardAndCheckDoubleSupport(SmoothICPComputer smoothICPComputer, double doubleSupportDuration, double initialTime,
         double omega0, Point3d initialICPPosition, Point3d transferFromFoot, Point3d icpPosition, Vector3d icpVelocity)
   {
      double deltaT = 0.001;
      Point3d previousICPPosition = new Point3d(initialICPPosition);
      for (double time = initialTime + deltaT; time <= initialTime + doubleSupportDuration; time = time + deltaT)
      {
         smoothICPComputer.getICPPositionAndVelocity(icpPosition, icpVelocity, omega0, time);

         Vector3d approximateVelocity = new Vector3d(icpPosition);
         approximateVelocity.sub(previousICPPosition);
         approximateVelocity.scale(1.0 / deltaT);

         JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocity, 1e-4);
         previousICPPosition.set(icpPosition);
      }
   }

   private void assertICPPointsAreAlongLineIncludingTransferFromFoot(Point3d transferFromFoot, Point3d initialICPPosition, Point3d icpPosition)
   {
       boolean pointsInOrderAndColinear = GeometryTools.arePointsInOrderAndColinear(transferFromFoot, initialICPPosition, icpPosition, 1e-4);
       
      if (!pointsInOrderAndColinear)
      {
         fail();
      }
   }

   @Test
   public void testTrivialTwoStepExample()
   {
      int maxNumberOfConsideredFootsteps = 4;
      double doubleSupportFirstStepFraction = 0.55;
      SmoothICPComputer smoothICPComputer = new SmoothICPComputer(doubleSupportFirstStepFraction , maxNumberOfConsideredFootsteps);

      ArrayList<Point3d> footLocations = new ArrayList<Point3d>();
      footLocations.add(new Point3d(0.0, 0.0, 0.0));
      footLocations.add(new Point3d(1.0, 1.0, 0.0));

      double singleSupportDuration = 0.5;
      double doubleSupportDuration = 0.2;
      double doubleSupportInitialTransferDuration = 0.3;

      double initialTime = 1.0;
      double omega0 = 0.33;
      
      smoothICPComputer.initializeDoubleSupportInitialTransfer(footLocations, singleSupportDuration, doubleSupportDuration,
              doubleSupportInitialTransferDuration, initialTime);

      Point3d icpPositionToPack = new Point3d();
      Vector3d icpVelocityToPack = new Vector3d();

      double time = initialTime;
      smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, omega0, time);

      Point3d averagePoint = new Point3d(footLocations.get(0));
      averagePoint.add(footLocations.get(1));

      averagePoint.scale(0.5);

      JUnitTools.assertTuple3dEquals(averagePoint, icpPositionToPack, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(), icpVelocityToPack, 1e-7);

      time = initialTime + doubleSupportInitialTransferDuration;
      smoothICPComputer.getICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, omega0, time);

      JUnitTools.assertTuple3dEquals(averagePoint, icpPositionToPack, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(), icpVelocityToPack, 1e-7);
   }




}
