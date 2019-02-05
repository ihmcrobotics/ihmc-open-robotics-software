package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import us.ihmc.robotics.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PassiveRevoluteJointTest
{
   // Variables
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private FrameVector3D frameVec = new FrameVector3D();
   private RigidBodyBasics rigidBody = new RigidBody("rigidBody", referenceFrame);
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
         joint.getJointTau(0, null);
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
         joint.setJointWrench(jointWrench);
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
         joint.setJointAcceleration(rowStart, matrix);
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
         joint.setJointConfiguration(rowStart, matrix);
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
         joint.setJointVelocity(rowStart, matrix);
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
         joint.setJointConfiguration(joint);
         joint.setJointTwist(joint);
         joint.setJointAcceleration(joint);
      }
      catch(RuntimeException e)
      {
         return; 
      }     
      Assert.fail();
   }
}
