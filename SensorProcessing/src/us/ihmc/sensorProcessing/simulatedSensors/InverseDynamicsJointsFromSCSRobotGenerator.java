package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class InverseDynamicsJointsFromSCSRobotGenerator
{   
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();
   
   private final Robot robot;
   private final RigidBody elevator;
   
   //TODO: Add SliderJoints
   public InverseDynamicsJointsFromSCSRobotGenerator(Robot robot)
   {
      this.robot = robot;
            
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);

      ConcurrentLinkedQueue<Joint> jointQueue = new ConcurrentLinkedQueue<Joint>();
      jointQueue.addAll(robot.getRootJoints());
            
      while(!jointQueue.isEmpty())
      {
         Joint polledJoint = jointQueue.poll();
         
         // Link parameters:
         Link link = polledJoint.getLink();  

         Matrix3d momentOfInertia = new Matrix3d();
         link.getMomentOfInertia(momentOfInertia);
         
         double mass = link.getMass();
         
         Vector3d comOffset = new Vector3d();
         link.getComOffset(comOffset);
                  
         RigidBody parentIDBody = getParentIDBody(polledJoint, elevator);
         
         if (polledJoint instanceof FloatingJoint)
         {
            FloatingJoint currentJoint = (FloatingJoint) polledJoint;

            FloatingInverseDynamicsJoint currentIDJoint = new SixDoFJoint(currentJoint.getName(), elevator, ReferenceFrame.getWorldFrame());
            ScrewTools.addRigidBody(currentJoint.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            scsToInverseDynamicsJointMap.addLinkedJoints(currentJoint, currentIDJoint);
         }
         else if (polledJoint instanceof PinJoint)
         {
            PinJoint currentJoint = (PinJoint) polledJoint;
            
            Vector3d jointAxis = new Vector3d();
            currentJoint.getJointAxis(jointAxis);
            
            Vector3d jointOffset = new Vector3d();
            currentJoint.getOffset(jointOffset); 
    
            RevoluteJoint currentIDJoint = ScrewTools.addRevoluteJoint(currentJoint.getName(), parentIDBody, jointOffset, jointAxis);
            
            ScrewTools.addRigidBody(currentJoint.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            scsToInverseDynamicsJointMap.addLinkedJoints(currentJoint, currentIDJoint);
         }
         else
         {
            throw new RuntimeException();
         }
         
         
         jointQueue.addAll(polledJoint.getChildrenJoints());
      }
      
      
      elevator.updateFramesRecursively();
   }
   
   public SCSToInverseDynamicsJointMap getSCSToInverseDynamicsJointMap()
   {
      return scsToInverseDynamicsJointMap;
   }
   
   public RigidBody getElevator()
   {
      return elevator;
   }
   
   
   private FloatingInverseDynamicsJoint getRootSixDoFJoint()
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() > 1) throw new RuntimeException("Only works with one root joint");
      
      FloatingJoint rootJoint = (FloatingJoint) rootJoints.get(0);
      
      return scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(rootJoint);
   }
   
   private RigidBody getRootBody()
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() > 1) throw new RuntimeException("Only works with one root joint");
      
      Joint rootJoint = rootJoints.get(0);
      
      return scsToInverseDynamicsJointMap.getRigidBody(rootJoint);
   }
  
   private RigidBody getParentIDBody(Joint polledJoint, RigidBody elevator)
   {
      Joint parentJoint = polledJoint.getParentJoint();

      if (parentJoint == null) return elevator;
      
      return scsToInverseDynamicsJointMap.getRigidBody(parentJoint);
   }

   private final SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector();
   
   private final FrameVector linearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector angularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
     
   private final FrameVector originAcceleration = new FrameVector();
   private final FrameVector angularAcceleration = new FrameVector();
   
   private final RigidBodyTransform positionAndRotation = new RigidBodyTransform();

   public void updateInverseDynamicsRobotModelFromRobot(boolean updateRootJoints, boolean updateDesireds)
   {
      // First update joint angles:
      Collection<OneDegreeOfFreedomJoint> pinJoints = scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints(); //pinToRevoluteJointMap.keySet();
      for (OneDegreeOfFreedomJoint pinJoint : pinJoints)
      {
         if (updateRootJoints || (pinJoint.getParentJoint() != null))
         {
            OneDoFJoint revoluteJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(pinJoint); //pinToRevoluteJointMap.get(pinJoint);

            double jointPosition = pinJoint.getQYoVariable().getDoubleValue();
            double jointVelocity = pinJoint.getQDYoVariable().getDoubleValue();
            double jointAcceleration = pinJoint.getQDDYoVariable().getDoubleValue();
            revoluteJoint.setQ(jointPosition);
            revoluteJoint.setQd(jointVelocity);

            if (updateDesireds)
               revoluteJoint.setQddDesired(jointAcceleration);
            else
               revoluteJoint.setQdd(jointAcceleration);
         }
      }
      
      if (updateRootJoints)
      {
         Collection<? extends FloatingJoint> floatingJoints = scsToInverseDynamicsJointMap.getFloatingJoints(); //floatingToSixDofToJointMap.keySet();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            FloatingInverseDynamicsJoint sixDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(floatingJoint); //floatingToSixDofToJointMap.get(floatingJoint);

            floatingJoint.getTransformToWorld(positionAndRotation);
            sixDoFJoint.setPositionAndRotation(positionAndRotation);
         }
      }
      
      // Then update all the frames. They will be needed for further computation:
      elevator.updateFramesRecursively();

     
      if (updateRootJoints)
      {
         Collection<? extends FloatingJoint> floatingJoints = scsToInverseDynamicsJointMap.getFloatingJoints(); 
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            FloatingInverseDynamicsJoint sixDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(floatingJoint);
            //                     referenceFrames.updateFrames();

            ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
            ReferenceFrame pelvisFrame = sixDoFJoint.getFrameAfterJoint();

            floatingJoint.getVelocity(linearVelocity);
            linearVelocity.changeFrame(pelvisFrame);

            floatingJoint.getAngularVelocity(angularVelocity, pelvisFrame);

            Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, linearVelocity.getVector(), angularVelocity.getVector());
            sixDoFJoint.setJointTwist(bodyTwist);

            // Acceleration:
            //Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use a  SixDoFJointSpatialAccelerationCalculator instead.

            originAcceleration.setToZero(sixDoFJoint.getFrameBeforeJoint());
            angularAcceleration.setToZero(sixDoFJoint.getFrameAfterJoint());
            
            floatingJoint.getLinearAccelerationInWorld(originAcceleration.getVector());
            floatingJoint.getAngularAccelerationInBody(angularAcceleration.getVector());
            originAcceleration.changeFrame(sixDoFJoint.getFrameBeforeJoint());

            spatialAccelerationVector.setToZero(sixDoFJoint.getFrameAfterJoint(), sixDoFJoint.getFrameBeforeJoint(), sixDoFJoint.getFrameAfterJoint());
            spatialAccelerationVector.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
            
            if (updateDesireds)
               sixDoFJoint.setDesiredAcceleration(spatialAccelerationVector);
            else
               sixDoFJoint.setAcceleration(spatialAccelerationVector);
         }
      }
   }
   
   public FullInverseDynamicsStructure getInverseDynamicsStructure()
   {
      RigidBody elevator = getElevator();

      FloatingInverseDynamicsJoint rootInverseDynamicsJoint = getRootSixDoFJoint();
      RigidBody estimationLink = getRootBody();

      return new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);
   }
   
}
