package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class SmoothenedConstantCoPICPTrajectoryGeneratorTest
{

   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(12525L);
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z};

      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);

      YoVariableRegistry parentRegistry = new YoVariableRegistry("test");
      int lookahead = 4;
      SmoothenedConstantCoPICPTrajectoryGenerator trajectoryGenerator = new SmoothenedConstantCoPICPTrajectoryGenerator(parentRegistry, lookahead);

      int nFootsteps = 15;
      List<Footstep> footsteps = new ArrayList<Footstep>();

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody rigidBody = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      ReferenceFrame soleFrame = rigidBody.getBodyFixedFrame();
      ContactablePlaneBody endEffector = new RectangularContactableBody(rigidBody, soleFrame, 1.0, -1.0, 1.0, -1.0);

      // Generate footsteps
      double steplength = 0.3; 
      double halfstepwidth = 0.1; 
      int flip = 1; 
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());

         //         pose.setPosition(new FramePoint(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));

         FramePoint framepoint = new FramePoint(ReferenceFrame.getWorldFrame()); 
         framepoint.set(i*steplength, flip*halfstepwidth, 0); 
         pose.setPosition(framepoint);

         PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame" + i, pose);
         boolean trustHeight = true;
         Footstep footstep = new Footstep(endEffector.getRigidBody(), null, soleFrame, poseFrame, trustHeight);
         footsteps.add(footstep);       

         flip = -flip; 
      }

      double w0 = 2.4;
      double singleSupportTime = 0.6;
      double doubleSupportTime = 0.2;
      double transferIntoWalkingTime = 0.4;
      double steppingTime = singleSupportTime + doubleSupportTime; 

      double currentTime = 0.3; 
      boolean isSingeSupport = !true; 

      List<FramePoint2d> equivalentConstantCoPs = new ArrayList<FramePoint2d>();
      List<FramePoint2d> desiredICPWaypoints = new ArrayList<FramePoint2d>();

      // compute lookahead (number) equivalent constant CoPs, desired ICP Waypoints from the equivalent constant CoPs
      trajectoryGenerator.compute(footsteps, w0, singleSupportTime, doubleSupportTime, transferIntoWalkingTime);

      // get equivalent constant CoPs
      equivalentConstantCoPs = trajectoryGenerator.getEquivalentConstantCoPs(footsteps, lookahead); 

      // get desired ICP Waypoints 
      desiredICPWaypoints = trajectoryGenerator.getDesiredICPWaypoints(equivalentConstantCoPs, w0, steppingTime); 

      // compute the polynomial parameter matrix, which represents the smooth double-support trajectory
      DenseMatrix64F paramMatrix = trajectoryGenerator.computeDoubleSupportpolynomialParams(equivalentConstantCoPs, desiredICPWaypoints, w0, steppingTime, doubleSupportTime); 

      // initialize desired ICP, desired ICP velocity and desired equivalent CoP
      FramePoint2d desiredICP = new FramePoint2d(ReferenceFrame.getWorldFrame());
      FrameVector2d desiredICPVelocity = new FrameVector2d(ReferenceFrame.getWorldFrame()); 
      FramePoint2d desiredEquivalentConstantCOP = new FramePoint2d(ReferenceFrame.getWorldFrame());

      // Compute desired ICP, desired ICP velocity and desired equivalent CoP (depending of the current time)
      trajectoryGenerator.calcDCMandECMPofTime(equivalentConstantCoPs, desiredICPWaypoints, w0, currentTime, steppingTime, doubleSupportTime, 
            isSingeSupport, desiredICP, desiredICPVelocity, desiredEquivalentConstantCOP, paramMatrix);

      int a; 
      a = 0; 
   }
}
