package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class PassiveRevoluteJointTest
{
   // Variables
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private FrameVector frameVec = new FrameVector();
   private RigidBody rigidBody = new RigidBody("rigidBody", referenceFrame);
   private PassiveRevoluteJoint joint = new PassiveRevoluteJoint("testJoint",rigidBody, referenceFrame, frameVec);
   private DenseMatrix64F matrix = new DenseMatrix64F();
   private int rowStart = 1;
   private Wrench jointWrench = new Wrench();
   private double q, qd, qdd, qddDesired, tau;
   
   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackTauMatrix()
   {
      try
      {
         joint.packTauMatrix(null);
      }
      catch(RuntimeException e)
      {
         return; // it caught an exception (which is what we wanted) so it returns (passes)
      }     
      Assert.fail(); // if it doesn't catch anything it will reach this line, which will make the test fail
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackVelocityMatrix()
   {
      try
      {
         joint.packVelocityMatrix(matrix, rowStart);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackDesiredAccelerationMatrix()
   {
      try
      {
         joint.packDesiredAccelerationMatrix(matrix, rowStart);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetTorqueFromWrench()
   {
      try
      {
         joint.setTorqueFromWrench(jointWrench);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetDesiredAcceleration()
   {
      try
      {
         joint.setDesiredAcceleration(matrix, rowStart);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
}
