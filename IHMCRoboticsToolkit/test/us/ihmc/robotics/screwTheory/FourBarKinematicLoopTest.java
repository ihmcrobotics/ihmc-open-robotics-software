package us.ihmc.robotics.screwTheory;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoopTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private RigidBody elevator;
   private ReferenceFrame elevatorFrame;
   
//   private FourBarKinematicLoop createRandomFourBarKinematicLoop(Random random)
//   {      
//      Matrix3d randomRotation = RandomTools.generateRandomRotationMatrix3d(random);
//      AxisAngle4d randomAxisAngle = new AxisAngle4d();
//      randomAxisAngle.set(randomRotation);
//      Vector3d jointAxis = new Vector3d(randomAxisAngle.x, randomAxisAngle.y, randomAxisAngle.z);
//      
//      elevatorFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame", random, worldFrame);
//      elevator = new RigidBody("elevator", elevatorFrame);
//      
//      RigidBodyTransform elevatorToMasterTransform = RigidBodyTransform.generateRandomTransform(random);
//      RevoluteJoint masterJointA = ScrewTools.addRevoluteJoint("masterJointA", elevator, elevatorToMasterTransform, jointAxis);
//
////      FourBarKinematicLoop fourBarKinematicLoop = new FourBarKinematicLoop("fourBarTestLoop", masterJointA, passiveJointB, passiveJointC, passiveJointD, closurePointFromLastPassiveJointInWorld, recomputeJointLimits);
//   }
}
