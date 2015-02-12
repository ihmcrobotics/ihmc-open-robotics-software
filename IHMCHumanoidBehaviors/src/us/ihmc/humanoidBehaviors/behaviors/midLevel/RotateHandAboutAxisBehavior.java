package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

/**
 * Rotate a 1-DOF body such as a valve or door handle/hinge by a specified angle.
 * Inputs: Grasp Pose and PinJoint Pose
 * @author cschmidt
 *
 */
public class RotateHandAboutAxisBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final TaskExecutor taskExecutor;
   private final HandPoseBehavior handPoseBehavior;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable hasInputBeenSet;

   public RotateHandAboutAxisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.taskExecutor = new TaskExecutor();
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet())
      {
         taskExecutor.doControl();
      }
   }

   public void setInput(RobotSide robotSide, RigidBodyTransform graspedObjectTransformToWorld, Axis pinJointAxisInGraspedObjectFrame,
         FramePose initialHandPoseInWorld, double turnAngleRad, double trajectoryTime)
   {
      PoseReferenceFrame initialGraspedObjectFrame = new PoseReferenceFrame("graspedObjectFrameBeforeRotation", world);
      initialGraspedObjectFrame.setPoseAndUpdate(graspedObjectTransformToWorld);

      int numberOfHandPoses = 5;
      double turnAnglePerPose = turnAngleRad / numberOfHandPoses;
      ArrayList<FramePose> desiredHandPoses = new ArrayList<FramePose>(numberOfHandPoses);
      
      //TODO: Reduce number of objects created in methods below
      desiredHandPoses.add( copyCurrentPoseAndRotateAboutPinJointAxis(initialHandPoseInWorld, initialGraspedObjectFrame,
            pinJointAxisInGraspedObjectFrame, turnAnglePerPose) );
      
      for (int i = desiredHandPoses.size(); i<numberOfHandPoses; i++)
      {
         desiredHandPoses.add(i, copyCurrentPoseAndRotateAboutPinJointAxis(desiredHandPoses.get(i-1), initialGraspedObjectFrame,
               pinJointAxisInGraspedObjectFrame, turnAnglePerPose));
      }

      for (FramePose desiredHandPose : desiredHandPoses)
      {
         taskExecutor.submit(new HandPoseTask(createHandPosePacket(robotSide, trajectoryTime, desiredHandPose), handPoseBehavior, yoTime));

      }

      hasInputBeenSet.set(true);
   }

   private FramePose copyCurrentPoseAndRotateAboutPinJointAxis(FramePose initialHandPoseInWorld, ReferenceFrame initialGraspedObjectFrame,
         Axis pinJointAxisInGraspedObjectFrame, double turnAngleRad)
   {
      RigidBodyTransform initialHandToGraspedObjectTransform = getHandToGraspedObjectTransform(initialHandPoseInWorld, initialGraspedObjectFrame);
      RigidBodyTransform pinJointRotation = createPinJointRotationTransform(pinJointAxisInGraspedObjectFrame, turnAngleRad);
      RigidBodyTransform desiredHandToGraspedObjectTransform = new RigidBodyTransform(initialHandToGraspedObjectTransform);
      desiredHandToGraspedObjectTransform.multiply(pinJointRotation);
      FramePose desiredHandPoseInWorld = new FramePose(initialGraspedObjectFrame, desiredHandToGraspedObjectTransform);

      setTranslationDueToOffsetRotationAxis(initialHandToGraspedObjectTransform, pinJointRotation, desiredHandPoseInWorld);

      desiredHandPoseInWorld.changeFrame(world);

      return desiredHandPoseInWorld;
   }

   private void setTranslationDueToOffsetRotationAxis(RigidBodyTransform initialHandToGraspedObjectTransform, RigidBodyTransform pinJointRotation,
         FramePose desiredHandPoseInWorld)
   {
      Vector3d desiredGraspedObjectToHandVector = new Vector3d();
      initialHandToGraspedObjectTransform.get(desiredGraspedObjectToHandVector);
      pinJointRotation.transform(desiredGraspedObjectToHandVector);
      desiredHandPoseInWorld.setPosition(desiredGraspedObjectToHandVector);
   }

   private RigidBodyTransform getHandToGraspedObjectTransform(FramePose handPoseInWorld, ReferenceFrame graspedObjectFrame)
   {
      FramePose initialHandPoseInGraspedObjectFrame = new FramePose(handPoseInWorld);
      initialHandPoseInGraspedObjectFrame.changeFrame(graspedObjectFrame);

      RigidBodyTransform initialHandToGraspedObjectTransform = new RigidBodyTransform();
      initialHandPoseInGraspedObjectFrame.getPose(initialHandToGraspedObjectTransform);

      return initialHandToGraspedObjectTransform;
   }

   private RigidBodyTransform createPinJointRotationTransform(Axis pinJointAxisInGraspedObjectFrame, double turnAngleRad)
   {
      RigidBodyTransform pinJointRotation = new RigidBodyTransform();
      if (pinJointAxisInGraspedObjectFrame == Axis.X)
      {
         pinJointRotation.rotX(turnAngleRad);
      }
      else if (pinJointAxisInGraspedObjectFrame == Axis.Y)
      {
         pinJointRotation.rotY(turnAngleRad);
      }
      else
      {
         pinJointRotation.rotZ(turnAngleRad);
      }

      return pinJointRotation;
   }

   private HandPosePacket createHandPosePacket(RobotSide robotSideOfHandToUse, double trajectoryTime, FramePose desiredHandPoseInWorld)
   {
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();

      desiredHandPoseInWorld.getPosition(position);
      desiredHandPoseInWorld.getOrientation(orientation);

      Frame packetFrame;
      ReferenceFrame referenceFrame = desiredHandPoseInWorld.getReferenceFrame();
      if (referenceFrame.isWorldFrame())
      {
         packetFrame = Frame.WORLD;
      }
      else
      {
         throw new RuntimeException("Hand Pose must be defined in World reference frame.");
      }

      HandPosePacket ret = new HandPosePacket(robotSideOfHandToUse, packetFrame, position, orientation, trajectoryTime);
      return ret;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      handPoseBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return taskExecutor.isDone();
   }

   @Override
   public void finalize()
   {
      hasInputBeenSet.set(false);

      handPoseBehavior.finalize();
   }

   @Override
   public void initialize()
   {
      handPoseBehavior.initialize();
      hasInputBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
