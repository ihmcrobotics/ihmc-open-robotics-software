package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.Assert;

public class PassiveRevoluteJointTest
{
   // Variables
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private FrameVector3D frameVec = new FrameVector3D();
   private RigidBodyBasics rigidBody = new RigidBody("rigidBody", referenceFrame);
   private PassiveRevoluteJoint joint = null; // new PassiveRevoluteJoint("testJoint",rigidBody, referenceFrame, frameVec);
   private DMatrixRMaj matrix = new DMatrixRMaj();
   private int rowStart = 1;
   private Wrench jointWrench = new Wrench();
   private double q, qd, qdd, qddDesired, tau;
   private boolean integrateQddDes;
   
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
   
   @Test
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
