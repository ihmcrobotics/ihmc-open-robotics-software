package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.ConstantVelocityTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class BacklashCompensatingVelocityYoVariableTest
{
   private static final double EPSILON = 1e-8;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testWithoutBacklashOrFiltering1()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      BacklashCompensatingVelocityYoVariable unprocessed = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, dt, slopTime, registry);

      double rawPosition = 0.0, rawPositionPrevValue = 0.0;
      unprocessed.update(rawPosition);

      for (int i = 0; i < 1000; i++)
      {
         rawPosition = RandomNumbers.nextDouble(rand, -100.0, 100.0);
         unprocessed.update(rawPosition);

         double rawVelocity = (rawPosition - rawPositionPrevValue) / dt;

         assertEquals(rawVelocity, unprocessed.getDoubleValue(), EPSILON);

         rawPositionPrevValue = rawPosition;
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testWithoutBacklashOrFiltering2()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      BacklashCompensatingVelocityYoVariable unprocessed = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
            registry);

      double rawPositionPrevValue = 0.0;
      unprocessed.update();

      for (int i = 0; i < 1000; i++)
      {
         rawPosition.set(RandomNumbers.nextDouble(rand, -100.0, 100.0));
         unprocessed.update();

         double rawVelocity = (rawPosition.getDoubleValue() - rawPositionPrevValue) / dt;

         assertEquals(rawVelocity, unprocessed.getDoubleValue(), EPSILON);

         rawPositionPrevValue = rawPosition.getDoubleValue();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testWithoutBacklash1()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      FilteredVelocityYoVariable filtVelocity = new FilteredVelocityYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable filteredOnly = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, dt, slopTime, registry);

      filtVelocity.update();
      filteredOnly.update(rawPosition.getDoubleValue());

      for (int i = 0; i < 1000; i++)
      {
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
         rawPosition.set(RandomNumbers.nextDouble(rand, -100.0, 100.0));
         filtVelocity.update();
         filteredOnly.update(rawPosition.getDoubleValue());

         assertEquals(filtVelocity.getDoubleValue(), filteredOnly.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testWithoutBacklash2()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      FilteredVelocityYoVariable filtVelocity = new FilteredVelocityYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable filteredOnly = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
            registry);

      filtVelocity.update();
      filteredOnly.update();

      for (int i = 0; i < 1000; i++)
      {
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
         rawPosition.set(RandomNumbers.nextDouble(rand, -100.0, 100.0));
         filtVelocity.update();
         filteredOnly.update();

         assertEquals(filtVelocity.getDoubleValue(), filteredOnly.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testVelocityPositiveWithoutCrossingZero2()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      FilteredVelocityYoVariable filtVelocity = new FilteredVelocityYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable backlashAndFiltered = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
            registry);

      filtVelocity.update();
      backlashAndFiltered.update();

      for (int i = 0; i < 1000; i++)
      {
         slopTime.set(RandomNumbers.nextDouble(rand, 0.0, 100.0));
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
         rawPosition.add(RandomNumbers.nextDouble(rand, 0.0, 101.0));
         filtVelocity.update();
         backlashAndFiltered.update();

         assertEquals(filtVelocity.getDoubleValue(), backlashAndFiltered.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testVelocityNegativeWithoutCrossingZero2()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      FilteredVelocityYoVariable filtVelocity = new FilteredVelocityYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable backlashAndFiltered = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
            registry);

      filtVelocity.update();
      backlashAndFiltered.update();

      for (int i = 0; i < 1000; i++)
      {
         slopTime.set(RandomNumbers.nextDouble(rand, 0.0, 100.0));
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
         rawPosition.sub(RandomNumbers.nextDouble(rand, 0.0, 101.0));
         filtVelocity.update();
         backlashAndFiltered.update();

         assertEquals(filtVelocity.getDoubleValue(), backlashAndFiltered.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.7)
	@Test(timeout=300000)
   public void testBacklashOnlyCrossingZeroConstantPositiveAcceleration2()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      double dt = 0.001;
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      BacklashCompensatingVelocityYoVariable backlashOnly = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
            registry);

      for (int i = 0; i < 100; i++)
      {
         slopTime.set(RandomNumbers.nextDouble(rand, 0.0, 0.9));
         double crossZeroAtT = RandomNumbers.nextDouble(rand, 3.0, 9.0);
         double trajectoryTime = RandomNumbers.nextDouble(rand, 2.0, 10.0);
         DoubleTrajectoryGenerator trajectory = createConstantAccelerationTrajectoryCrossingZeroAtCrossTime(rand, crossZeroAtT, trajectoryTime, true);
         trajectory.compute(0.0);
         double initialPosition = RandomNumbers.nextDouble(rand, -10000.0, 10000.0);
         rawPosition.set(initialPosition);
         double rawPositionPrevValue = initialPosition;
         backlashOnly.set(0.0);
         backlashOnly.reset();
         backlashOnly.update();
         double crossTime = Double.NaN;

         for (double t = 0.0; t < trajectoryTime; t += dt)
         {
            trajectory.compute(t);
            backlashOnly.update();

            double rawVelocity = (rawPosition.getDoubleValue() - rawPositionPrevValue) / dt;
            double velocity;
            if (rawVelocity > 0.0)
            {
               if (Double.isNaN(crossTime))
               {
                  crossTime = t;
               }

               double timeInSlop = t - crossTime;
               double percent = timeInSlop / slopTime.getDoubleValue();
               percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
               if (Double.isNaN(percent))
               {
                  percent = 1.0;
               }
               velocity = percent * rawVelocity;
            }
            else
            {
               velocity = rawVelocity;
            }

            assertEquals("At iteration: " + i + ", t = " + t + "; ", velocity, backlashOnly.getDoubleValue(), EPSILON);

            rawPositionPrevValue = rawPosition.getDoubleValue();
            rawPosition.add(trajectory.getValue() * dt);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
	@Test(timeout=300000)
   public void testBacklashOnlyCrossingZeroConstantNegativeAcceleration2()
   {
      Random rand = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      double dt = 0.001; //1e-5; //RandomTools.generateRandomDouble(rand, 1e-8, 1.0);
      DoubleYoVariable slopTime = new DoubleYoVariable("slop", registry);
      DoubleYoVariable rawPosition = new DoubleYoVariable("rawPosition", registry);
      BacklashCompensatingVelocityYoVariable backlashOnly = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
            registry);

      for (int i = 0; i < 100; i++)
      {
         slopTime.set(RandomNumbers.nextDouble(rand, 0.0, 0.9));
         double crossZeroAtT = RandomNumbers.nextDouble(rand, 3.0, 9.0);
         double trajectoryTime = RandomNumbers.nextDouble(rand, 2.0, 10.0);
         DoubleTrajectoryGenerator trajectory = createConstantAccelerationTrajectoryCrossingZeroAtCrossTime(rand, crossZeroAtT, trajectoryTime, false);
         trajectory.compute(0.0);
         double initialPosition = RandomNumbers.nextDouble(rand, -10000.0, 10000.0);
         rawPosition.set(initialPosition);
         double rawPositionPrevValue = initialPosition;
         backlashOnly.set(0.0);
         backlashOnly.reset();
         backlashOnly.update();
         double crossTime = Double.NaN;

         for (double t = 0.0; t < trajectoryTime; t += dt)
         {
            trajectory.compute(t);
            backlashOnly.update();

            double rawVelocity = (rawPosition.getDoubleValue() - rawPositionPrevValue) / dt;
            double velocity;
            if (rawVelocity < 0.0)
            {
               if (Double.isNaN(crossTime))
               {
                  crossTime = t;
               }

               double timeInSlop = t - crossTime;
               double percent = timeInSlop / slopTime.getDoubleValue();
               percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
               if (Double.isNaN(percent))
               {
                  percent = 1.0;
               }
               velocity = percent * rawVelocity;
            }
            else
            {
               velocity = rawVelocity;
            }

            assertEquals("At iteration: " + i + ", t = " + t + "; ", velocity, backlashOnly.getDoubleValue(), EPSILON);

            rawPositionPrevValue = rawPosition.getDoubleValue();
            rawPosition.add(trajectory.getValue() * dt);
         }
      }
   }

	
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testNoisySignalAndMakeSureVelocityHasSignalContent()
   {
      Random random = new Random(1798L);

      YoVariableRegistry registry = new YoVariableRegistry("Registry");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      DoubleYoVariable slopTime = new DoubleYoVariable("slopTime", registry);
      DoubleYoVariable cleanPosition = new DoubleYoVariable("cleanPosition", registry);
      DoubleYoVariable noisyPosition = new DoubleYoVariable("noisyPosition", registry);
      DoubleYoVariable cleanVelocity = new DoubleYoVariable("cleanVelocity", registry);

      
      DoubleYoVariable reconstructedPosition = new DoubleYoVariable("reconstructedPosition", registry);
      DoubleYoVariable reconstructedPosition2 = new DoubleYoVariable("reconstructedPosition2", registry);

      DoubleYoVariable totalReconstructedPositionError1 = new DoubleYoVariable("totalReconstructedPositionError1", registry);
      DoubleYoVariable totalReconstructedPositionError2 = new DoubleYoVariable("totalReconstructedPositionError2", registry);
      
      DoubleYoVariable averageReconstructedPositionError1 = new DoubleYoVariable("averageReconstructedPositionError1", registry);
      DoubleYoVariable averageReconstructedPositionError2 = new DoubleYoVariable("averageReconstructedPositionError2", registry);

      double dt = 0.001;
      double totalTime = 5.0;

      double amplitude = 2.0;
      double frequency = 1.0;
      double noiseAmplitude = 0.01;
      
      slopTime.set(0.1);
      alphaVariable.set(0.95);
      
      BacklashCompensatingVelocityYoVariable backlashCompensatingVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_velocity", "", alphaVariable, noisyPosition, dt, slopTime, registry);
      RevisedBacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity = new RevisedBacklashCompensatingVelocityYoVariable("bl_qd_velocity2", "", alphaVariable, noisyPosition, dt, slopTime, registry);

      reconstructedPosition.set(amplitude);
      reconstructedPosition2.set(amplitude);
      
//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();
      
      for (double time = 0.0; time < totalTime; time = time + dt)
      {
         cleanPosition.set(amplitude * Math.cos(2.0 * Math.PI * frequency * time)); 
         cleanVelocity.set(-2.0 * Math.PI * amplitude * frequency * Math.sin(2.0 * Math.PI * frequency * time));
         
         noisyPosition.set(cleanPosition.getDoubleValue());
         noisyPosition.add(RandomNumbers.nextDouble(random, noiseAmplitude));
         
         backlashCompensatingVelocity.update();
         revisedBacklashCompensatingVelocity.update();
         
         reconstructedPosition.add(backlashCompensatingVelocity.getDoubleValue() * dt);
         reconstructedPosition2.add(revisedBacklashCompensatingVelocity.getDoubleValue() * dt);
         
        double positionError1 = reconstructedPosition.getDoubleValue() - cleanPosition.getDoubleValue();
        totalReconstructedPositionError1.add(Math.abs(positionError1) * dt);
        
        double positionError2 = reconstructedPosition2.getDoubleValue() - cleanPosition.getDoubleValue();
        totalReconstructedPositionError2.add(Math.abs(positionError2) * dt);
                
//        scs.tickAndUpdate();
      }
      
      averageReconstructedPositionError1.set(totalReconstructedPositionError1.getDoubleValue() / totalTime);
      averageReconstructedPositionError2.set(totalReconstructedPositionError2.getDoubleValue() / totalTime);
      
      // The original one doesn't do very well with noisy signals because it thinks the noise is backlash.
      assertFalse(averageReconstructedPositionError1.getDoubleValue() < 0.25);
      assertTrue(averageReconstructedPositionError2.getDoubleValue() < 0.25);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSignalWithBacklash()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Registry");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      DoubleYoVariable slopTime = new DoubleYoVariable("slopTime", registry);
      
      DoubleYoVariable cleanPosition = new DoubleYoVariable("cleanPosition", registry);
      DoubleYoVariable backlashyPosition = new DoubleYoVariable("backlashyPosition", registry);
      DoubleYoVariable cleanVelocity = new DoubleYoVariable("cleanVelocity", registry);

      
      DoubleYoVariable reconstructedPosition = new DoubleYoVariable("reconstructedPosition", registry);
      DoubleYoVariable reconstructedPosition2 = new DoubleYoVariable("reconstructedPosition2", registry);

      DoubleYoVariable totalReconstructedPositionError1 = new DoubleYoVariable("totalReconstructedPositionError1", registry);
      DoubleYoVariable totalReconstructedPositionError2 = new DoubleYoVariable("totalReconstructedPositionError2", registry);
      
      DoubleYoVariable averageReconstructedPositionError1 = new DoubleYoVariable("averageReconstructedPositionError1", registry);
      DoubleYoVariable averageReconstructedPositionError2 = new DoubleYoVariable("averageReconstructedPositionError2", registry);

      double dt = 0.001;
      double totalTime = 5.0;

      double amplitude = 2.0;
      double frequency = 1.0;
      double backlashAmount = 0.1;
      
      slopTime.set(0.1);
      alphaVariable.set(0.95);
      
      BacklashCompensatingVelocityYoVariable backlashCompensatingVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_velocity", "", alphaVariable, backlashyPosition, dt, slopTime, registry);
      RevisedBacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity = new RevisedBacklashCompensatingVelocityYoVariable("bl_qd_velocity2", "", alphaVariable, backlashyPosition, dt, slopTime, registry);

      reconstructedPosition.set(amplitude);
      reconstructedPosition2.set(amplitude);
      
//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();
      
      for (double time = 0.0; time < totalTime; time = time + dt)
      {
         cleanPosition.set(amplitude * Math.cos(2.0 * Math.PI * frequency * time)); 
         cleanVelocity.set(-2.0 * Math.PI * amplitude * frequency * Math.sin(2.0 * Math.PI * frequency * time));
         
         backlashyPosition.set(cleanPosition.getDoubleValue());
         if(cleanVelocity.getDoubleValue() > 0.0)
         {
            backlashyPosition.add(backlashAmount);
         }
         
         backlashCompensatingVelocity.update();
         revisedBacklashCompensatingVelocity.update();
         
         reconstructedPosition.add(backlashCompensatingVelocity.getDoubleValue() * dt);
         reconstructedPosition2.add(revisedBacklashCompensatingVelocity.getDoubleValue() * dt);
         
        double positionError1 = reconstructedPosition.getDoubleValue() - cleanPosition.getDoubleValue();
        totalReconstructedPositionError1.add(Math.abs(positionError1) * dt);
        
        double positionError2 = reconstructedPosition2.getDoubleValue() - cleanPosition.getDoubleValue();
        totalReconstructedPositionError2.add(Math.abs(positionError2) * dt);
                
//        scs.tickAndUpdate();
      }
      
      averageReconstructedPositionError1.set(totalReconstructedPositionError1.getDoubleValue() / totalTime);
      averageReconstructedPositionError2.set(totalReconstructedPositionError2.getDoubleValue() / totalTime);
      
      assertTrue(averageReconstructedPositionError2.getDoubleValue() < 0.25);
   }
   
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testRemoveSquareWaveBacklash()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Registry");
      DoubleYoVariable alphaVariable = new DoubleYoVariable("alpha", registry);
      DoubleYoVariable slopTime = new DoubleYoVariable("slopTime", registry);
      
      DoubleYoVariable backlashyPosition = new DoubleYoVariable("backlashyPosition", registry);

      double dt = 0.001;
      double totalTime = 5.0;

      double frequency = 30.0;
      double backlashAmount = 0.1;
      
      slopTime.set(0.1);
      alphaVariable.set(0.95);
      
      BacklashCompensatingVelocityYoVariable backlashCompensatingVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_velocity", "", alphaVariable, backlashyPosition, dt, slopTime, registry);
      RevisedBacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity = new RevisedBacklashCompensatingVelocityYoVariable("bl_qd_velocity2", "", alphaVariable, backlashyPosition, dt, slopTime, registry);
      
//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();
      
      for (double time = 0.0; time < totalTime; time = time + dt)
      {
         backlashyPosition.set(Math.cos(2.0 * Math.PI * frequency * time)); 
         if (backlashyPosition.getDoubleValue() > 0.0) 
         {
            backlashyPosition.set(backlashAmount);
         }
         else
         {
            backlashyPosition.set(-backlashAmount);
         }
         
         backlashCompensatingVelocity.update();
         revisedBacklashCompensatingVelocity.update();
         
         assertEquals(0.0, revisedBacklashCompensatingVelocity.getDoubleValue(), 1e-3);
                
//        scs.tickAndUpdate();
      }
   }
	  
	  
	  
	  
   private DoubleTrajectoryGenerator createConstantAccelerationTrajectoryCrossingZeroAtCrossTime(Random rand, double crossTime, double trajectoryTime,
         boolean positiveVelocity)
   {
      double initialPositionSign = positiveVelocity ? -1.0 : 1.0;
      DoubleProvider initialPositionProvider = new ConstantDoubleProvider(initialPositionSign * RandomNumbers.nextDouble(rand, 0.1, 1000.0));
      DoubleProvider velocityProvider = new ConstantDoubleProvider(-initialPositionProvider.getValue() / crossTime);
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      YoVariableRegistry registry = new YoVariableRegistry("osenroi");
      ConstantVelocityTrajectoryGenerator trajectory = new ConstantVelocityTrajectoryGenerator("hihi", initialPositionProvider, velocityProvider,
            trajectoryTimeProvider, registry);
      trajectory.initialize();
      return trajectory;
   }
}
