package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public class CoMHeightTimeDerivativesSmootherTest
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testConstantHeight()
   {
      double dt = 0.001;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      CoMHeightTimeDerivativesSmoother smoother = new CoMHeightTimeDerivativesSmoother(dt, registry);

      CoMHeightTimeDerivativesData comHeightDataOut = new CoMHeightTimeDerivativesData();
      CoMHeightTimeDerivativesData comHeightDataIn = new CoMHeightTimeDerivativesData();

      double comHeight = 1.2;
      double comHeightVelocity = 0.0;
      double comHeightAcceleration = 0.0;

      comHeightDataIn.setComHeight(ReferenceFrame.getWorldFrame(), comHeight);
      comHeightDataIn.setComHeightVelocity(comHeightVelocity);
      comHeightDataIn.setComHeightAcceleration(comHeightAcceleration);

      smoother.initialize(comHeightDataIn);
      smoother.smooth(comHeightDataOut, comHeightDataIn);

      FramePoint comHeightPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      comHeightDataOut.getComHeight(comHeightPoint);
      double comHeightOut = comHeightPoint.getZ();
      double comHeightVelocityOut = comHeightDataOut.getComHeightVelocity();
      double comHeightAccelerationOut = comHeightDataOut.getComHeightAcceleration();

      assertEquals(comHeight, comHeightOut, 1e-7);
      assertEquals(comHeightVelocity, comHeightVelocityOut, 1e-7);
      assertEquals(comHeightAcceleration, comHeightAccelerationOut, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testDiscreetJump()
   {
      boolean visualize = false;

      double dt = 0.002;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      DoubleYoVariable testTime = new DoubleYoVariable("testTime", registry);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Null"));
         scs.addYoVariableRegistry(registry);
         scs.startOnAThread();
      }

      CoMHeightTimeDerivativesSmoother smoother = new CoMHeightTimeDerivativesSmoother(dt, registry);

      CoMHeightTimeDerivativesData comHeightDataOut = new CoMHeightTimeDerivativesData();
      CoMHeightTimeDerivativesData comHeightDataIn = new CoMHeightTimeDerivativesData();

      double comHeightIn = 1.0;
      double comHeightVelocityIn = 0.0;
      double comHeightAccelerationIn = 0.0;

      comHeightDataIn.setComHeight(worldFrame, comHeightIn);
      comHeightDataIn.setComHeightVelocity(comHeightVelocityIn);
      comHeightDataIn.setComHeightAcceleration(comHeightAccelerationIn);

      smoother.initialize(comHeightDataIn);
      smoother.smooth(comHeightDataOut, comHeightDataIn);

      FramePoint comHeightPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      comHeightDataOut.getComHeight(comHeightPoint);
      double comHeightOut = comHeightPoint.getZ();
      double comHeightVelocityOut = comHeightDataOut.getComHeightVelocity();
      double comHeightAccelerationOut = comHeightDataOut.getComHeightAcceleration();

      assertEquals(comHeightIn, comHeightOut, 1e-7);
      assertEquals(comHeightVelocityIn, comHeightVelocityOut, 1e-7);
      assertEquals(comHeightAccelerationIn, comHeightAccelerationOut, 1e-7);

      comHeightIn = 1.2;
      comHeightDataIn.setComHeight(worldFrame, comHeightIn);

      double previousCoMHeightOut = comHeightOut;
      double previousCoMHeightVelocityOut = comHeightVelocityOut;

      double estimatedZDot, estimatedZDDot;
      if (visualize)
      {
         scs.updateAndTick();
      }

      for (int i = 0; i < (1.5 / dt); i++)
      {
         testTime.add(dt);
         smoother.smooth(comHeightDataOut, comHeightDataIn);

         comHeightDataOut.getComHeight(comHeightPoint);
         double newComHeightOut = comHeightPoint.getZ();
         estimatedZDot = (newComHeightOut - previousCoMHeightOut) / dt;
         estimatedZDDot = (estimatedZDot - previousCoMHeightVelocityOut) / dt;

         assertEquals(estimatedZDot, comHeightDataOut.getComHeightVelocity(), 1e-7);
         assertEquals(estimatedZDDot, comHeightDataOut.getComHeightAcceleration(), 1e-7);
         //         comHeightDataIn.set(comHeightDataOut);

         previousCoMHeightOut = newComHeightOut;
         previousCoMHeightVelocityOut = comHeightDataOut.getComHeightVelocity();

         if (visualize)
         {
            scs.tickAndUpdate();
         }
      }

      comHeightDataOut.getComHeight(comHeightPoint);
      double finalComHeightOut = comHeightPoint.getZ();

      assertEquals(comHeightIn, finalComHeightOut, 1e-4);
      assertEquals(comHeightVelocityIn, comHeightDataOut.getComHeightVelocity(), 1e-3);
      assertEquals(comHeightAccelerationIn, comHeightDataOut.getComHeightAcceleration(), 1e-2);

      if (visualize)
      {
         scs.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testSinusoidalInput()
   {
      boolean visualize = false;

      double dt = 0.002;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      DoubleYoVariable testTime = new DoubleYoVariable("testTime", registry);

      DoubleYoVariable amplitude = new DoubleYoVariable("amplitude", registry);
      DoubleYoVariable frequency = new DoubleYoVariable("frequency", registry);

      amplitude.set(0.2);
      frequency.set(1.0);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Null"));
         scs.addYoVariableRegistry(registry);
         scs.startOnAThread();
      }

      CoMHeightTimeDerivativesSmoother smoother = new CoMHeightTimeDerivativesSmoother(dt, registry);

      CoMHeightTimeDerivativesData comHeightDataOut = new CoMHeightTimeDerivativesData();
      CoMHeightTimeDerivativesData comHeightDataIn = new CoMHeightTimeDerivativesData();

      double comHeightIn = 0.0; //amplitude.getDoubleValue(); //1.0;
      double comHeightVelocityIn = 2.0 * Math.PI * frequency.getDoubleValue() * amplitude.getDoubleValue();
      double comHeightAccelerationIn = 0.0;

      comHeightDataIn.setComHeight(worldFrame, comHeightIn);
      comHeightDataIn.setComHeightVelocity(comHeightVelocityIn);
      comHeightDataIn.setComHeightAcceleration(comHeightAccelerationIn);

      smoother.initialize(comHeightDataIn);
      smoother.smooth(comHeightDataOut, comHeightDataIn);

      if (visualize)
      {
         scs.updateAndTick();
      }

      boolean done = false;
      while (!done)
      {
         testTime.add(dt);

         double twoPIFreq = 2.0 * Math.PI * frequency.getDoubleValue();
         comHeightIn = amplitude.getDoubleValue() * Math.sin(twoPIFreq * testTime.getDoubleValue());
         comHeightVelocityIn = twoPIFreq * amplitude.getDoubleValue() * Math.cos(twoPIFreq * testTime.getDoubleValue());
         comHeightAccelerationIn = -twoPIFreq * twoPIFreq * amplitude.getDoubleValue() * Math.sin(twoPIFreq * testTime.getDoubleValue());

         comHeightDataIn.setComHeight(worldFrame, comHeightIn);
         comHeightDataIn.setComHeightVelocity(comHeightVelocityIn);
         comHeightDataIn.setComHeightAcceleration(comHeightAccelerationIn);

         smoother.smooth(comHeightDataOut, comHeightDataIn);

         if (visualize)
         {
            //            ThreadTools.sleep((long) (dt * 1000));
            scs.tickAndUpdate();
         }
         //         else
         {
            if (testTime.getDoubleValue() > 3.0)
               done = true;
         }
      }

      if (visualize)
      {
         scs.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

}
