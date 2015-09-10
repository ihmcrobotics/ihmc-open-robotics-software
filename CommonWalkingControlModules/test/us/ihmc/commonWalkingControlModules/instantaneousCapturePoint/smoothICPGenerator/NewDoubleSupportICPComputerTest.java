package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class NewDoubleSupportICPComputerTest
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeICPCornerPoints()
   {
      boolean visualize = false;

      NewDoubleSupportICPComputer newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();

      ArrayList<Point3d> footLocations = new ArrayList<Point3d>();
      footLocations.add(new Point3d(0.0, 0.0, 0.0));
      footLocations.add(new Point3d(0.21, 0.22, 0.0));
      footLocations.add(new Point3d(0.02, 0.42, 0.0));
      footLocations.add(new Point3d(0.2, 0.63, 0.0));

      PointAndLinePlotter pointAndLinePlotter = null;
      SimulationConstructionSet scs = null;
      if (visualize)
      {
         Robot robot = new Robot(getClass().getSimpleName());
         scs = new SimulationConstructionSet(robot);
         pointAndLinePlotter = new PointAndLinePlotter(robot.getRobotsYoVariableRegistry());
         pointAndLinePlotter.createAndShowOverheadPlotterInSCS(scs);

         pointAndLinePlotter.plotPoints("footLocations", footLocations, YoAppearance.Black(), 0.01);
      }

      double singleSupportDuration = 0.6;
      double doubleSupportDuration = 0.2;

      double omega0 = 3.0;

      int numberOfCornerPoints = footLocations.size() - 1;
      Point3d[] icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, footLocations,
                                     singleSupportDuration + doubleSupportDuration, omega0);

      if (visualize)
      {
         pointAndLinePlotter.plotPoint3ds("cornerPoints", icpCornerPoints, YoAppearance.Green(), 0.01);
      }

      assertEquals(numberOfCornerPoints, icpCornerPoints.length);

      for (int index = 0; index < numberOfCornerPoints - 1; index++)
      {
         Point3d footLocation = footLocations.get(index);
         Point3d firstCorner = icpCornerPoints[index];
         Point3d secondCorner = icpCornerPoints[index + 1];

         assertTrue(GeometryTools.arePointsInOrderAndColinear(footLocation, firstCorner, secondCorner, 1e-4));
      }

      assertTrue(GeometryTools.arePointsInOrderAndColinear(footLocations.get(numberOfCornerPoints - 1), icpCornerPoints[numberOfCornerPoints - 1],
              footLocations.get(numberOfCornerPoints), 1e-4));


      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();

         ThreadTools.sleepForever();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeSingleSupportICP()
   {
      boolean visualize = false;

      NewDoubleSupportICPComputer newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();

      Point3d supportFoot = new Point3d(0.1, 0.2, 0.3);
      Point3d cornerPoint0 = new Point3d(0.3, 0.4, 0.5);

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

         pointAndLinePlotter.plotPoint3d("supportFoot", supportFoot, YoAppearance.Black(), 0.01);
         pointAndLinePlotter.plotPoint3d("cornerPoint0", cornerPoint0, YoAppearance.Green(), 0.01);
      }
      
      double omega0 = 0.7;
      double singleSupportDuration = 0.7;
      double doubleSupportDuration = 1.1;

      double doubleSupportFirstStepFraction = 0.5;

      Point3d icpPosition = new Point3d();
      Vector3d icpVelocity = new Vector3d();

      Point3d singleSupportStartICP = newDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
                                         doubleSupportFirstStepFraction, omega0);
      Point3d singleSupportEndICP = newDoubleSupportICPComputer.computeSingleSupportEndICP(supportFoot, cornerPoint0, doubleSupportDuration,
                                       doubleSupportFirstStepFraction, singleSupportDuration, omega0);

      if (visualize)
      {
         pointAndLinePlotter.plotPoint3d("singleSupportStartICP", singleSupportStartICP, YoAppearance.Cyan(), 0.01);
         pointAndLinePlotter.plotPoint3d("singleSupportEndICP", singleSupportEndICP, YoAppearance.Cyan(), 0.01);
      }
      
      assertTrue(GeometryTools.arePointsInOrderAndColinear(supportFoot, cornerPoint0, singleSupportStartICP, 1e-4));
      assertTrue(GeometryTools.arePointsInOrderAndColinear(cornerPoint0, singleSupportStartICP, singleSupportEndICP, 1e-4));

      Point3d initialICPPosition = new Point3d(singleSupportStartICP);
      Point3d previousICPPosition = new Point3d(singleSupportStartICP);

      YoFramePoint icpArrowTip = null;
      YoFramePoint icpPositionYoFramePoint = null;
      YoFrameLineSegment2d icpVelocityLineSegment = null;
      if (visualize)
      {
         icpArrowTip = new YoFramePoint("icpVelocityTip", "", worldFrame, registry);
         icpVelocityLineSegment = new YoFrameLineSegment2d("icpVelocity", "", worldFrame, registry);
         icpPositionYoFramePoint = new YoFramePoint("icpPosition", "", worldFrame, registry);
         
         pointAndLinePlotter.plotYoFramePoint("icpPosition", icpPositionYoFramePoint, YoAppearance.OrangeRed(), 0.01);
         pointAndLinePlotter.plotLineSegment("icpVelocity", icpVelocityLineSegment, Color.gray);
      }
      
      double deltaT = 0.001;
      for (double time = deltaT; time <= singleSupportDuration; time = time + deltaT)
      {
         newDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPosition, icpVelocity, supportFoot, singleSupportStartICP, omega0, time);

         if (visualize)
         {
            icpPositionYoFramePoint.set(icpPosition);
            PointAndLinePlotter.setEndPointGivenStartAndAdditionalVector(icpArrowTip, icpPosition, icpVelocity, 0.5);
            Point2d icpPosition2d = new Point2d(icpPosition.getX(), icpPosition.getY());
            PointAndLinePlotter.setLineSegmentBasedOnStartAndEndFramePoints(icpVelocityLineSegment, icpPosition2d,
                  icpArrowTip.getFramePoint2dCopy().getPointCopy());
            
            scs.tickAndUpdate();
         }
         
         assertTrue(GeometryTools.arePointsInOrderAndColinear(supportFoot, initialICPPosition, icpPosition, 1e-4));

         Vector3d approximateVelocity = new Vector3d(icpPosition);
         approximateVelocity.sub(previousICPPosition);
         approximateVelocity.scale(1.0 / deltaT);

         JUnitTools.assertTuple3dEquals(approximateVelocity, icpVelocity, 1e-3);
         previousICPPosition.set(icpPosition);
      }

      JUnitTools.assertTuple3dEquals(singleSupportEndICP, icpPosition, 1e-3);
      
      if (visualize)
      {
         pointAndLinePlotter.addPointsAndLinesToSCS(scs);

         scs.startOnAThread();

         ThreadTools.sleepForever();
      }
   }


}
