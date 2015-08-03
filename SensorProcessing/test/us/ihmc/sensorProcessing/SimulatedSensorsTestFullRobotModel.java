package us.ihmc.sensorProcessing;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.code.visualization.Visualize;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.AbstractInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;

@Visualize
public class SimulatedSensorsTestFullRobotModel
{
   private final RigidBody elevator;
   private final RigidBody body;

   private final SixDoFJoint rootJoint;
   private final ReferenceFrame worldFrame;

   public SimulatedSensorsTestFullRobotModel()
   {
      worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());

      elevator = new RigidBody("elevator", elevatorFrame);

      rootJoint = new SixDoFJoint("imu", elevator, elevatorFrame);    // origin is at the IMU
      Matrix3d momentOfInertia = new Matrix3d();
      momentOfInertia.setM00(1.0);
      momentOfInertia.setM11(1.0);
      momentOfInertia.setM22(0.1);
  
      body = ScrewTools.addRigidBody("body", rootJoint, momentOfInertia, 1.0, new Vector3d(0.0, 0.0, 0.0));
   }

   public void update(SingleRigidBodyRobot robot)
   {
      // Update Body Pose
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      robot.getTransformToWorld(transformToWorld);
      rootJoint.setPositionAndRotation(transformToWorld);
      updateFrames();

      // Update Body Velocity
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame imuFrame = rootJoint.getFrameAfterJoint();

      FrameVector linearVelocity = robot.getBodyVelocity();
      linearVelocity.changeFrame(imuFrame);

      FrameVector angularVelocity = robot.getBodyAngularVelocityInBodyFrame(imuFrame);
      angularVelocity.changeFrame(imuFrame);

      Twist bodyTwist = new Twist(imuFrame, elevatorFrame, imuFrame, linearVelocity.getVector(), angularVelocity.getVector());
      rootJoint.setJointTwist(bodyTwist);

      // Update Body Acceleration
      FrameVector linearAccelerationOfOrigin = robot.getBodyAcceleration();
      FrameVector angularAcceleration = robot.getBodyAngularAccelerationInBodyFrame(imuFrame);
      SpatialAccelerationVector accelerationOfChestWithRespectToWorld = new SpatialAccelerationVector(imuFrame, elevatorFrame, imuFrame);
      accelerationOfChestWithRespectToWorld.setBasedOnOriginAcceleration(angularAcceleration, linearAccelerationOfOrigin, bodyTwist);
      rootJoint.setAcceleration(accelerationOfChestWithRespectToWorld);
   }
   
   public void updateFrames()
   {
      elevator.updateFramesRecursively();
   }

   public ReferenceFrame getElevatorFrame()
   {
      return elevator.getBodyFixedFrame();
   }
   
   public ArrayList<ReferenceFrame> getAllReferenceFrames()
   {
      ArrayList<InverseDynamicsJoint> jointStack = new ArrayList<InverseDynamicsJoint>();
      jointStack.add(rootJoint);
      
      ArrayList<ReferenceFrame> ret = new ArrayList<ReferenceFrame>();
      while (!jointStack.isEmpty())
      {
         InverseDynamicsJoint currentJoint = jointStack.remove(0);
         ret.add(currentJoint.getFrameAfterJoint());
         jointStack.addAll(currentJoint.getSuccessor().getChildrenJoints());
      }
      
      ret.add(worldFrame);

      return ret;
   }

   public AbstractInverseDynamicsJoint getRootJoint()
   {
      return rootJoint;
   }

   public RigidBody getElevator()
   {
      return elevator;
   }
   
   public RigidBody getBodyLink()
   {
      return body;
   }
}

