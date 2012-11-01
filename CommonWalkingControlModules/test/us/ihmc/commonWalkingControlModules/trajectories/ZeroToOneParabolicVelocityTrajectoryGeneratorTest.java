package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ZeroToOneParabolicVelocityTrajectoryGeneratorTest
{
   private double trajectoryTime;
   private ZeroToOneParabolicVelocityTrajectoryGenerator trajectoryGenerator;
   private double epsilon;
   
   @Before
   public void setUp() throws Exception
   {
      trajectoryTime = 1.0;
      trajectoryGenerator = new ZeroToOneParabolicVelocityTrajectoryGenerator("", trajectoryTime, new YoVariableRegistry("test"));
      trajectoryGenerator.initialize();
      epsilon = 1e-7;
   }

   @Test
   public void testStartValue()
   {
      trajectoryGenerator.compute(0.0);
      
      assertEquals(0.0, trajectoryGenerator.getValue(), epsilon);
   }
   
   @Test
   public void testEndValue()
   {
      trajectoryGenerator.compute(trajectoryTime);
      
      assertEquals(1.0, trajectoryGenerator.getValue(), epsilon);
   }
   
   @Test
   public void testMidValue()
   {
      trajectoryGenerator.compute(trajectoryTime / 2.0);
      
      assertEquals(0.5, trajectoryGenerator.getValue(), epsilon);
   }

   @Test
   public void testAfterEndValue()
   {
      trajectoryGenerator.compute(trajectoryTime * 1.5);
      
      assertEquals(1.0, trajectoryGenerator.getValue(), epsilon);
   }
   
   @Test
   public void testIsDone()
   {
      trajectoryGenerator.compute(0.0);
      assertFalse(trajectoryGenerator.isDone());
      
      trajectoryGenerator.compute(0.5 * trajectoryTime);
      assertFalse(trajectoryGenerator.isDone());

      trajectoryGenerator.compute(1.0 * trajectoryTime);
      assertTrue(trajectoryGenerator.isDone());

      trajectoryGenerator.compute(1.5 * trajectoryTime);
      assertTrue(trajectoryGenerator.isDone());
   }
}
