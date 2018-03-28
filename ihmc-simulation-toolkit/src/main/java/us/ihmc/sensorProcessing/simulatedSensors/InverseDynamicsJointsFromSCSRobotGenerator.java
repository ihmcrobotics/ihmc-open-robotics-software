package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class InverseDynamicsJointsFromSCSRobotGenerator
{   
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = new SCSToInverseDynamicsJointMap();
   
   private final Robot robot;
   private final RigidBody elevator;
   
   //TODO: Add SliderJoints
   public InverseDynamicsJointsFromSCSRobotGenerator(Robot robot)
   {
      this.robot = robot;
            
      elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());

      ConcurrentLinkedQueue<Joint> jointQueue = new ConcurrentLinkedQueue<Joint>();
      jointQueue.addAll(robot.getRootJoints());
            
      while(!jointQueue.isEmpty())
      {
         Joint polledJoint = jointQueue.poll();
         
         // Link parameters:
         Link link = polledJoint.getLink();  

         Matrix3D momentOfInertia = new Matrix3D();
         link.getMomentOfInertia(momentOfInertia);
         
         double mass = link.getMass();
         
         Vector3D comOffset = new Vector3D();
         link.getComOffset(comOffset);
                  
         RigidBody parentIDBody = getParentIDBody(polledJoint, elevator);
         
         if (polledJoint instanceof FloatingJoint)
         {
            FloatingJoint currentJoint = (FloatingJoint) polledJoint;

            FloatingInverseDynamicsJoint currentIDJoint = new SixDoFJoint(currentJoint.getName(), elevator);
            ScrewTools.addRigidBody(currentJoint.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            scsToInverseDynamicsJointMap.addLinkedJoints(currentJoint, currentIDJoint);
         }
         else if (polledJoint instanceof PinJoint)
         {
            PinJoint currentJoint = (PinJoint) polledJoint;
            
            Vector3D jointAxis = new Vector3D();
            currentJoint.getJointAxis(jointAxis);
            
            Vector3D jointOffset = new Vector3D();
            currentJoint.getOffset(jointOffset); 
    
            RevoluteJoint currentIDJoint = ScrewTools.addRevoluteJoint(currentJoint.getName(), parentIDBody, jointOffset, jointAxis);
            currentIDJoint.setJointLimitLower(currentJoint.getJointLowerLimit());
            currentIDJoint.setJointLimitUpper(currentJoint.getJointUpperLimit());

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
   
   private final FrameVector3D linearVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D angularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
     
   private final FrameVector3D originAcceleration = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();
   
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

            Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, linearVelocity, angularVelocity);
            sixDoFJoint.setJointTwist(bodyTwist);
            sixDoFJoint.updateFramesRecursively();

            // Acceleration:
            //Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use a  SixDoFJointSpatialAccelerationCalculator instead.

            originAcceleration.setToZero(sixDoFJoint.getFrameBeforeJoint());
            angularAcceleration.setToZero(sixDoFJoint.getFrameAfterJoint());
            
            floatingJoint.getLinearAccelerationInWorld(originAcceleration);
            floatingJoint.getAngularAccelerationInBody(angularAcceleration);
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

   /**
    * updates an SCS Robot with the results from the Inverse Dynamics Model
    * @param updateRootJoints update from Inverse Dynamics Model
    * @param updateAccelerations update from Inverse Dynamics Model
    * @param useDesiredAcceleration should SCS robot acceleration should be updated from the Inverse Dynamics Model
    * @param updatePositions update from Inverse Dynamics Model
    * @param updateVelocities update from Inverse Dynamics Model
    * @param updateTorques update from Inverse Dynamics Model
    * @param useDesiredTorque should torques be copied to the SCS Robot
    */
   public void updateRobotFromInverseDynamicsRobotModel(boolean updateRootJoints, boolean updateAccelerations, boolean useDesiredAcceleration, boolean updatePositions, boolean updateVelocities, boolean updateTorques, boolean useDesiredTorque)
   {

      Collection<OneDegreeOfFreedomJoint> pinJoints = scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints(); //pinToRevoluteJointMap.keySet();
      for (OneDegreeOfFreedomJoint pinJoint : pinJoints)
      {
         if (updateRootJoints || (pinJoint.getParentJoint() != null))
         {
            OneDoFJoint revoluteJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(pinJoint); //pinToRevoluteJointMap.get(pinJoint);

            double jointPosition;
            double jointVelocity;
            double jointAcceleration;
            double jointTorque;
            if (updateAccelerations)
            {
               if (useDesiredAcceleration)
                  jointAcceleration = revoluteJoint.getQddDesired();
               else
                  jointAcceleration = revoluteJoint.getQdd();
               pinJoint.setQdd(jointAcceleration);
            }

            if (updatePositions)
            {
               jointPosition = revoluteJoint.getQ();
               pinJoint.setQ(jointPosition);
            }

            if (updateVelocities)
            {
               jointVelocity = revoluteJoint.getQd();
               pinJoint.setQd(jointVelocity);
            }

            if (updateTorques)
            {
               if (useDesiredTorque)
                  jointTorque = revoluteJoint.getTauMeasured();
               else
                  jointTorque = revoluteJoint.getTau();
               pinJoint.setTau(jointTorque);
            }

         }
      }

      if (updateRootJoints)
      {
         Collection<? extends FloatingJoint> floatingJoints = scsToInverseDynamicsJointMap.getFloatingJoints(); //floatingToSixDofToJointMap.keySet();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            FloatingInverseDynamicsJoint sixDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsSixDoFJoint(floatingJoint); //floatingToSixDofToJointMap.get(floatingJoint);
            RigidBodyTransform rotationAndTranslation = new RigidBodyTransform();
            if (updatePositions)
            {
               sixDoFJoint.getJointTransform3D(rotationAndTranslation);
               floatingJoint.setRotationAndTranslation(rotationAndTranslation);
            }

            ReferenceFrame pelvisFrame = sixDoFJoint.getFrameAfterJoint();

            if (updateVelocities)
            {
               linearVelocity.setIncludingFrame(pelvisFrame, sixDoFJoint.getLinearVelocityForReading());
               linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
               floatingJoint.setVelocity(linearVelocity);
               floatingJoint.setAngularVelocityInBody(sixDoFJoint.getAngularVelocityForReading());
            }

            if (updateAccelerations)
            {
               if (useDesiredAcceleration)
                  sixDoFJoint.getDesiredJointAcceleration(spatialAccelerationVector);
               else
                  sixDoFJoint.getJointAcceleration(spatialAccelerationVector);
               spatialAccelerationVector.getLinearPart(originAcceleration);
               originAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
               floatingJoint.setAcceleration(originAcceleration);
               spatialAccelerationVector.getAngularPart(angularAcceleration);
               floatingJoint.setAngularAccelerationInBody(angularAcceleration);
            }
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
