package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

public class CoMHeightTimeDerivativesSmootherTest
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void showMemoryUsageAfterTest()
   {
      ReferenceFrameTools.clearWorldFrameTree();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testConstantHeight()
   {
      double dt = 0.001;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      CoMHeightTimeDerivativesSmoother smoother = new CoMHeightTimeDerivativesSmoother(dt, registry);

      CoMHeightTimeDerivativesData comHeightDataOut = new CoMHeightTimeDerivativesData("out", registry);
      CoMHeightTimeDerivativesData comHeightDataIn = new CoMHeightTimeDerivativesData("in", registry);

      double comHeight = 1.2;
      double comHeightVelocity = 0.0;
      double comHeightAcceleration = 0.0;

      comHeightDataIn.setComHeight(ReferenceFrame.getWorldFrame(), comHeight);
      comHeightDataIn.setComHeightVelocity(comHeightVelocity);
      comHeightDataIn.setComHeightAcceleration(comHeightAcceleration);

      smoother.initialize(comHeightDataIn);
      smoother.smooth(comHeightDataOut, comHeightDataIn);

      FramePoint3D comHeightPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
      comHeightDataOut.getComHeight(comHeightPoint);
      double comHeightOut = comHeightPoint.getZ();
      double comHeightVelocityOut = comHeightDataOut.getComHeightVelocity();
      double comHeightAccelerationOut = comHeightDataOut.getComHeightAcceleration();

      assertEquals(comHeight, comHeightOut, 1e-7);
      assertEquals(comHeightVelocity, comHeightVelocityOut, 1e-7);
      assertEquals(comHeightAcceleration, comHeightAccelerationOut, 1e-7);
   }

   @Test
   public void testDiscreetJump()
   {
      boolean visualize = false;

      double dt = 0.002;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoDouble testTime = new YoDouble("testTime", registry);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Null"));
         scs.addYoVariableRegistry(registry);
         scs.startOnAThread();
      }

      CoMHeightTimeDerivativesSmoother smoother = new CoMHeightTimeDerivativesSmoother(dt, registry);

      CoMHeightTimeDerivativesData comHeightDataOut = new CoMHeightTimeDerivativesData("out", registry);
      CoMHeightTimeDerivativesData comHeightDataIn = new CoMHeightTimeDerivativesData("in", registry);

      double comHeightIn = 1.0;
      double comHeightVelocityIn = 0.0;
      double comHeightAccelerationIn = 0.0;

      comHeightDataIn.setComHeight(worldFrame, comHeightIn);
      comHeightDataIn.setComHeightVelocity(comHeightVelocityIn);
      comHeightDataIn.setComHeightAcceleration(comHeightAccelerationIn);

      smoother.initialize(comHeightDataIn);
      smoother.smooth(comHeightDataOut, comHeightDataIn);

      FramePoint3D comHeightPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
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

   @Test
   public void testSinusoidalInput()
   {
      boolean visualize = false;

      double dt = 0.002;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoDouble testTime = new YoDouble("testTime", registry);

      YoDouble amplitude = new YoDouble("amplitude", registry);
      YoDouble frequency = new YoDouble("frequency", registry);

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

      CoMHeightTimeDerivativesData comHeightDataOut = new CoMHeightTimeDerivativesData("out", registry);
      CoMHeightTimeDerivativesData comHeightDataIn = new CoMHeightTimeDerivativesData("in", registry);

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
