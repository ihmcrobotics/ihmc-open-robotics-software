package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PassiveRevoluteJointTest
{
   // Variables
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private FrameVector frameVec = new FrameVector();
   private RigidBody rigidBody = new RigidBody("rigidBody", referenceFrame);
   private PassiveRevoluteJoint joint = null; // new PassiveRevoluteJoint("testJoint",rigidBody, referenceFrame, frameVec);
   private DenseMatrix64F matrix = new DenseMatrix64F();
   private int rowStart = 1;
   private Wrench jointWrench = new Wrench();
   private double q, qd, qdd, qddDesired, tau;
   private boolean integrateQddDes;
   
   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackTauMatrix()
   {
      try
      {
         joint.getTauMatrix(null);
      }
      catch(RuntimeException e)
      {
         return; // it caught an exception (which is what we wanted) so it returns (passes)
      }     
      Assert.fail(); // if it doesn't catch anything it will reach this line, which will make the test fail
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQ()
   {
      try
      {
         joint.setQ(q);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQd()
   {
      try
      {
         joint.setQd(qd);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQdd()
   {
      try
      {
         joint.setQdd(qdd);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQddDesired()
   {
      try
      {
         joint.setQddDesired(qddDesired);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetTau()
   {
      try
      {
         joint.setTau(tau);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetConfiguration()
   {
      try
      {
         joint.setConfiguration(matrix, rowStart);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetVelocity()
   {
      try
      {
         joint.setVelocity(matrix, rowStart);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetJointPositionVelocityAndAcceleration()
   {
      try
      {
         joint.setJointPositionVelocityAndAcceleration(joint);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
    
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetQddDesiredFromJoint()
   {
      try
      {
         joint.setQddDesired(joint);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
}
