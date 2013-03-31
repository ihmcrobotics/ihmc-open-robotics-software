package us.ihmc.commonWalkingControlModules.dynamics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJointSpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;

public class InverseDynamicsJointsFromSCSRobotGenerator
{   
   private final HashMap<SixDoFJoint, FloatingJoint> sixDofToFloatingJointMap = new HashMap<SixDoFJoint, FloatingJoint>();
   private final HashMap<FloatingJoint, SixDoFJoint> floatingToSixDofToJointMap = new HashMap<FloatingJoint, SixDoFJoint>();
   
   private final HashMap<RevoluteJoint, PinJoint> revoluteToPinJointMap = new HashMap<RevoluteJoint, PinJoint>();
   private final HashMap<PinJoint, RevoluteJoint> pinToRevoluteJointMap = new HashMap<PinJoint, RevoluteJoint>();
   
   private final Robot robot;
   private final RigidBody elevator;
   
   public InverseDynamicsJointsFromSCSRobotGenerator(Robot robot)
   {
      this.robot = robot;
      
      double gravity = robot.getGravityZ();
      
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new Transform3D());
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

            SixDoFJoint currentIDJoint = new SixDoFJoint(currentJoint.getName(), elevator, ReferenceFrame.getWorldFrame());            
            ScrewTools.addRigidBody(currentJoint.getName(), currentIDJoint, momentOfInertia, mass, comOffset);

            sixDofToFloatingJointMap.put(currentIDJoint, currentJoint);
            floatingToSixDofToJointMap.put(currentJoint, currentIDJoint);
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

            revoluteToPinJointMap.put(currentIDJoint, currentJoint);
            pinToRevoluteJointMap.put(currentJoint, currentIDJoint);
         }
         else
         {
            throw new RuntimeException();
         }
         
         
         jointQueue.addAll(polledJoint.getChildrenJoints());
      }
      
      
      elevator.updateFramesRecursively();
   }
   
   
   public RigidBody getElevator()
   {
      return elevator;
   }
   
   
   public SixDoFJoint getRootSixDoFJoint()
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() > 1) throw new RuntimeException("Only works with one root joint");
      
      Joint rootJoint = rootJoints.get(0);
      
      return floatingToSixDofToJointMap.get(rootJoint);
   }
   
   public RigidBody getRootBody()
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() > 1) throw new RuntimeException("Only works with one root joint");
      
      Joint rootJoint = rootJoints.get(0);
      
      return getRigidBody(rootJoint);
   }
  
   private RigidBody getParentIDBody(Joint polledJoint, RigidBody elevator)
   {
      Joint parentJoint = polledJoint.getParentJoint();

      if (parentJoint == null) return elevator;
      
      return getRigidBody(parentJoint);
   }
   
   public RigidBody getRigidBody(Joint joint)
   {      
      if (joint instanceof FloatingJoint)
      {
         SixDoFJoint parentSixDoFJoint = floatingToSixDofToJointMap.get(joint);
         return parentSixDoFJoint.getSuccessor();
      }
      else if (joint instanceof PinJoint)
      {
         RevoluteJoint parentRevoluteJoint = pinToRevoluteJointMap.get(joint);
         return parentRevoluteJoint.getSuccessor();
      }
      else
      {
         throw new RuntimeException();
      }   
   }

   private final SixDoFJointSpatialAccelerationCalculator sixDoFJointSpatialAccelerationCalculator = new SixDoFJointSpatialAccelerationCalculator();
   private final SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector();
   
   private final FrameVector linearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector angularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
     
   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularAcceleration = new Vector3d();
   
   private final Transform3D positionAndRotation = new Transform3D();

   public void updateInverseDynamicsRobotModelFromRobot(boolean updateRootJoints)
   {
      // First update joint angles:
      Set<PinJoint> pinJoints = pinToRevoluteJointMap.keySet();
      for (PinJoint pinJoint : pinJoints)
      {
         if (updateRootJoints || (pinJoint.getParentJoint() != null))
         {
            RevoluteJoint revoluteJoint = pinToRevoluteJointMap.get(pinJoint);

            double jointPosition = pinJoint.getQ().getDoubleValue();
            double jointVelocity = pinJoint.getQD().getDoubleValue();
            double jointAcceleration = pinJoint.getQDD().getDoubleValue();
            revoluteJoint.setQ(jointPosition);
            revoluteJoint.setQd(jointVelocity);
            revoluteJoint.setQddDesired(jointAcceleration);
         }
      }
      
      if (updateRootJoints)
      {
         Set<FloatingJoint> floatingJoints = floatingToSixDofToJointMap.keySet();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            SixDoFJoint sixDoFJoint = floatingToSixDofToJointMap.get(floatingJoint);

            floatingJoint.getTransformToWorld(positionAndRotation);
            sixDoFJoint.setPositionAndRotation(positionAndRotation);
         }
      }
      
      // Then update all the frames. They will be needed for further computation:
      elevator.updateFramesRecursively();

     
      if (updateRootJoints)
      {
         Set<FloatingJoint> floatingJoints = floatingToSixDofToJointMap.keySet();
         for (FloatingJoint floatingJoint : floatingJoints)
         {
            SixDoFJoint sixDoFJoint = floatingToSixDofToJointMap.get(floatingJoint);
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

            floatingJoint.getLinearAccelerationInWorld(linearAcceleration);
            floatingJoint.getAngularAccelerationInBody(angularAcceleration);

            sixDoFJointSpatialAccelerationCalculator.computeSpatialAccelerationInBodyFrame(spatialAccelerationVector, sixDoFJoint, linearAcceleration, angularAcceleration);
            sixDoFJoint.setDesiredAcceleration(spatialAccelerationVector);
         }
      }
   }

   public RevoluteJoint getInverseDynamicsRevoluteJoint(PinJoint pinJoint)
   {
      return pinToRevoluteJointMap.get(pinJoint);      
   }

   public Collection<PinJoint> getPinJoints()
   {
      return pinToRevoluteJointMap.keySet();
   }

   public SixDoFJoint getInverseDynamicsSixDoFJoint(FloatingJoint floatingJoint)
   {
      return floatingToSixDofToJointMap.get(floatingJoint);
   }

   public Collection<? extends FloatingJoint> getFloatingJoints()
   {
      return floatingToSixDofToJointMap.keySet();
   }

}
