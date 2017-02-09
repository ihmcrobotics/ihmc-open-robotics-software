package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript.HeadingAndVelocityEvaluationEvent;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class HeadingAndVelocityEvaluationScriptTest
{
   private static final boolean SHOW_GUI = false;

   private static final double HEADING_VIZ_Z = 0.03;
   private static final double VELOCITY_VIZ_Z = 0.06;

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

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testHeadingAndVelocityEvaluationScript()
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("HeadingAndVelocityEvaluationScriptTest");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double controlDT = 0.1;
      double desiredHeadingFinal = 0.0;

      SimpleDesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(desiredHeadingFinal, controlDT, parentRegistry);
      ManualDesiredVelocityControlModule desiredVelocityControlModule = new ManualDesiredVelocityControlModule(ReferenceFrame.getWorldFrame(), parentRegistry);
      HeadingAndVelocityEvaluationScriptParameters scriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
      boolean cycleThroughAllEvents = true;
      HeadingAndVelocityEvaluationScript script = new HeadingAndVelocityEvaluationScript(cycleThroughAllEvents, controlDT, desiredHeadingControlModule,
            desiredVelocityControlModule, scriptParameters, parentRegistry);

      double time = 0.0;

      YoFramePoint position = new YoFramePoint("position", "", ReferenceFrame.getWorldFrame(), parentRegistry);
      YoFrameVector velocity = new YoFrameVector("velocity", "", ReferenceFrame.getWorldFrame(), parentRegistry);
      YoFrameVector heading = new YoFrameVector("heading", "", ReferenceFrame.getWorldFrame(), parentRegistry);

      YoGraphicVector velocityVector = new YoGraphicVector("velocity", position, velocity, YoAppearance.Yellow());
      YoGraphicVector headingVector = new YoGraphicVector("heading", position, heading, YoAppearance.Blue());

      yoGraphicsListRegistry.registerYoGraphic("velocityVector", velocityVector);
      yoGraphicsListRegistry.registerYoGraphic("headingVector", headingVector);

      BagOfBalls bagOfBalls = new BagOfBalls(1200, 0.03, YoAppearance.Red(), parentRegistry, yoGraphicsListRegistry);

      boolean[] seenEvents = new boolean[HeadingAndVelocityEvaluationEvent.values().length];

      SimulationConstructionSet scs = null;
      if (SHOW_GUI)
         scs = setupAndStartSCS(parentRegistry, yoGraphicsListRegistry, controlDT);

      int numberOfTicksToTest = 1200;

      ArrayList<FrameVector2d> desiredHeadings = new ArrayList<FrameVector2d>();
      ArrayList<FrameVector2d> desiredVelocities = new ArrayList<FrameVector2d>();

      FrameVector2d desiredVelocity = new FrameVector2d();

      for (int i = 0; i < numberOfTicksToTest; i++)
      {
         script.update(time);

         desiredHeadingControlModule.updateDesiredHeadingFrame();

         FrameVector2d desiredHeading = new FrameVector2d();

         desiredHeadingControlModule.getDesiredHeading(desiredHeading, 0.0);
         desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
         double desiredHeadingAngle = desiredHeadingControlModule.getDesiredHeadingAngle();

         desiredHeadings.add(desiredHeading);
         desiredVelocities.add(desiredVelocity);

         double angleError = AngleTools.computeAngleDifferenceMinusPiToPi(desiredHeadingAngle, Math.atan2(desiredHeading.getY(), desiredHeading.getX()));
         assertTrue(Math.abs(angleError) < 1e-7);

         heading.set(desiredHeading.getX(), desiredHeading.getY(), HEADING_VIZ_Z);
         velocity.set(desiredVelocity.getX(), desiredVelocity.getY(), VELOCITY_VIZ_Z);

         position.add(desiredVelocity.getX() * controlDT, desiredVelocity.getY() * controlDT, 0.0);

         FramePoint location = new FramePoint(ReferenceFrame.getWorldFrame());
         location.set(position.getX(), position.getY(), 0.0);

         bagOfBalls.setBall(location);

         HeadingAndVelocityEvaluationEvent evaluationEvent = script.getEvaluationEvent();
         seenEvents[evaluationEvent.ordinal()] = true;

         time = time + controlDT;

         if (scs != null)
         {
            scs.setTime(time);
            scs.tickAndUpdate();
         }
      }

      for (boolean seenEvent : seenEvents)
      {
         assertTrue(seenEvent);
      }

      // Ensure that the maximum accelerations are within reasonable limits:

      double[] maxHeadingChanges = findMaxChange(desiredHeadings);
      double[] maxVelocityChanges = findMaxChange(desiredVelocities);

      for (int i = 0; i < 2; i++)
      {
         maxHeadingChanges[i] /= controlDT;
         maxVelocityChanges[i] /= controlDT;
      }

      //      System.out.println("maxHeadingChanges = " + maxHeadingChanges[0] + ", " + maxHeadingChanges[1]);
      //      System.out.println("maxVelocityChanges = " + maxVelocityChanges[0] + ", " + maxVelocityChanges[1]);
      //      System.out.println("desiredHeadingControlModule.getMaxHeadingDot() = " + desiredHeadingControlModule.getMaxHeadingDot());
      //      System.out.println("script.getAcceleration() = " + script.getAcceleration());

      assertTrue(maxHeadingChanges[0] < script.getMaxHeadingDot() * 1.05);
      assertTrue(maxHeadingChanges[1] < script.getMaxHeadingDot() * 1.05);

      assertTrue(maxVelocityChanges[0] < script.getAcceleration() * 2.0);
      assertTrue(maxVelocityChanges[1] < script.getAcceleration() * 2.0);

      if (SHOW_GUI)
         sleepForever();
   }

   private double[] findMaxChange(ArrayList<FrameVector2d> frameVectors)
   {
      double[] ret = new double[] {0.0, 0.0};

      FrameVector2d difference = new FrameVector2d(frameVectors.get(0).getReferenceFrame());

      FrameVector2d previousVector = frameVectors.get(0);

      for (int i = 1; i < frameVectors.size(); i++)
      {
         FrameVector2d nextVector = frameVectors.get(i);

         difference.set(nextVector);
         difference.sub(previousVector);

         if (Math.abs(difference.getX()) > ret[0])
         {
            ret[0] = difference.getX();
            //            System.out.println(i + ": difference = " + difference);
         }

         if (Math.abs(difference.getY()) > ret[1])
         {
            ret[1] = difference.getY();
            //            System.out.println(i + ": difference = " + difference);
         }

         previousVector = nextVector;
      }

      return ret;
   }

   private void sleepForever()
   {
      while (true)
      {
         sleep(1.0);
      }
   }

   private void sleep(double time)
   {
      try
      {
         Thread.sleep((long) (time * 1000));
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private SimulationConstructionSet setupAndStartSCS(YoVariableRegistry registryToWatch, YoGraphicsListRegistry yoGraphicsListRegistry, double controlDT)
   {
      Robot robot = new Robot("robot");

      robot.getRobotsYoVariableRegistry().addChild(registryToWatch);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(controlDT, 1);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Thread thread = new Thread(scs);
      thread.start();

      return scs;
   }

}
