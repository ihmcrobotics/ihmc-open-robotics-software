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
   DenseMatrix64F matrix = new DenseMatrix64F();
   ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   FrameVector frameVec = new FrameVector();
   RigidBody rigidBody = new RigidBody("rigidBody", referenceFrame);
   PassiveRevoluteJoint joint = new PassiveRevoluteJoint("testJoint",rigidBody, referenceFrame, frameVec);
   
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
         return;
      }     
      Assert.fail();
   }
   

}
