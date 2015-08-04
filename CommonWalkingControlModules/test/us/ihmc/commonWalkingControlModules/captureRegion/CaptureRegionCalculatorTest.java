package us.ihmc.commonWalkingControlModules.captureRegion;


import static org.junit.Assert.assertEquals;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.plotting.SimulationOverheadPlotter;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class CaptureRegionCalculatorTest
{
   private final boolean SHOW_GUI = false;
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testOne()
   {
      ReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
      ReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
      
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);
      
      double midFootAnkleXOffset = 0.15;
      double footWidth = 0.2;
      double kinematicRangeFromContactReferencePoint = 0.6;

      CapturePointCalculatorInterface capturePointCalculator = new SimpleCapturePointCalculator();
      YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("CaptureRegionCalculatorTest");

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      RobotSide supportSide = RobotSide.LEFT;

      CaptureRegionCalculator calculator = new CaptureRegionCalculator(ankleZUpFrames, midFootAnkleXOffset, footWidth, kinematicRangeFromContactReferencePoint, capturePointCalculator, yoVariableRegistry, yoGraphicsListRegistry);
      
      double swingTimeRemaining = 0.1;
      
      double[][] pointList = new double[][]{{-0.1, 0.1},{0.2, 0.1}, {0.2, -0.1}, {-0.1, -0.1}};
      FrameConvexPolygon2d supportFoot = new FrameConvexPolygon2d(ankleZUpFrames.get(supportSide), pointList);
      
      if (SHOW_GUI)
      {
         Robot robot = new Robot("Test");
         robot.addYoVariableRegistry(yoVariableRegistry);
         
         SimulationConstructionSet scs = new SimulationConstructionSet(robot);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
                 
         SimulationOverheadPlotter plotter = new SimulationOverheadPlotter();
         plotter.setDrawHistory(false);
         plotter.setXVariableToTrack(null);
         plotter.setYVariableToTrack(null);

         scs.attachPlaybackListener(plotter);
         JPanel r2Sim02PlotterJPanel = plotter.getJPanel();
         scs.addExtraJpanel(r2Sim02PlotterJPanel, "Plotter");
         JPanel r2Sim02PlotterKeyJPanel = plotter.getJPanelKey();

         JScrollPane scrollPane = new JScrollPane(r2Sim02PlotterKeyJPanel);

         scs.addExtraJpanel(scrollPane, "Plotter Legend");

         yoGraphicsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());

         scs.addExtraJpanel(plotter.getJPanel(), "Plotter");
         
         Thread thread = new Thread(scs);
         thread.start();
      }
      
      FrameConvexPolygon2d captureRegion = calculator.calculateCaptureRegion(supportSide, supportFoot, swingTimeRemaining);
      captureRegion.changeFrame(ReferenceFrame.getWorldFrame());
      
//      System.out.println("captureRegion = " + captureRegion);
      
      double[][] expectedPointList = new double[][]{
            {0.275, -0.175},
            {0.425, -0.175},
            {0.6149269713632459, -0.2021324244804637},
            {0.5853971601432783, -0.270831831420372},
            {0.5475514140914703, -0.3353246044291683},
            {0.5019775636657666, -0.39460902415271487},
            {0.4985268586524354, -0.3985268586524353},
            {0.4493834743506884, -0.44776426880175807},
            {0.39058605189741147, -0.49396471660730357},
            {0.32649855392065846, -0.5324927696032922},
            {0.275, -0.275}};

      FrameConvexPolygon2d expectedCaptureRegion = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), expectedPointList);

      assertEquals(expectedCaptureRegion.getNumberOfVertices(), captureRegion.getNumberOfVertices());
      
      for (int i=0; i<expectedCaptureRegion.getNumberOfVertices(); i++)
      {
         FramePoint2d expectedVertex = expectedCaptureRegion.getFrameVertex(i);
         
         FramePoint2d actualVertex = captureRegion.getClosestVertexCopy(expectedVertex);
                  
//         System.out.println("expected = " + expectedVertex);
//         System.out.println("actual = " + actualVertex);
         
         double distance = expectedVertex.distance(actualVertex);
         assertEquals(0.0, distance, 1e-7);
      }
      
      if (SHOW_GUI)
      {
         while(true)
         {
            try
            {
               Thread.sleep(10000);
            } catch (InterruptedException e)
            {
            }

         }
      }
   }

   private class  SimpleAnkleZUpReferenceFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -2855876641425187923L;
      private final Vector3d offset = new Vector3d();
      
      public SimpleAnkleZUpReferenceFrame(String name)
      {
         super(name, ReferenceFrame.getWorldFrame());
      }

      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setIdentity();
         transformToParent.setTranslation(offset);
      }
   }
   
   
   private class SimpleCapturePointCalculator implements CapturePointCalculatorInterface
   {
      FramePoint capturePoint = new FramePoint(ReferenceFrame.getWorldFrame(), 0.25, -0.15, 0.0);
      
      public void computeCapturePoint(RobotSide supportSide)
      {
         return;
      }

      public FramePoint computePredictedCapturePoint(RobotSide supportLeg, double captureTime, FramePoint centerOfPressure)
      {
         FrameVector coPToCapture = new FrameVector(capturePoint);
         coPToCapture.changeFrame(centerOfPressure.getReferenceFrame());
         coPToCapture.sub(centerOfPressure);
         coPToCapture.scale(0.5); // For the tests, just go 10% further than the capture point for the predicted capture point.
         
         FramePoint ret = new FramePoint(capturePoint);
         ret.changeFrame(centerOfPressure.getReferenceFrame());
         
         ret.add(coPToCapture);
         
         return ret;
      }

      public FramePoint getCapturePointInFrame(ReferenceFrame referenceFrame)
      {
          FramePoint ret = new FramePoint(capturePoint);
          ret.changeFrame(referenceFrame);
          
          return ret;
      }

      public FramePoint2d getCapturePoint2dInFrame(ReferenceFrame referenceFrame)
      {
         FramePoint2d ret = capturePoint.toFramePoint2d();
         
         ret.changeFrame(referenceFrame);
         
         return ret;
      }
      
   }
   
}
