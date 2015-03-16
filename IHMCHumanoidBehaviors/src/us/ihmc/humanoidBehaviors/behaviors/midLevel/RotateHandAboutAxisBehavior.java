package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseListTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyInverseKinematicTask;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
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
   private double radiansToRotateBetweenHandPoses = Math.toRadians(18.0);

   private final boolean DEBUG = false;
   private final boolean USE_WHOLE_BODY_IK;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final FullRobotModel fullRobotModel;

   private final TaskExecutor taskExecutor;
   private final HandPoseListBehavior handPoseListBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyInverseKinematicBehavior;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable hasInputBeenSet;

   public RotateHandAboutAxisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime,
         boolean useWholeBodyInverseKinematics)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;
      this.taskExecutor = new TaskExecutor();
      handPoseListBehavior = new HandPoseListBehavior(outgoingCommunicationBridge, yoTime);
      this.wholeBodyInverseKinematicBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters,
            fullRobotModel, yoTime);
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
      this.USE_WHOLE_BODY_IK = useWholeBodyInverseKinematics;
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet())
      {
         taskExecutor.doControl();
      }
   }

   public void setInput(RobotSide robotSide, Axis axisOrientationInGraspedObjectFrame, RigidBodyTransform graspedObjectTransformToWorld,
         double totalRotationInRadians, double rotationRateRadPerSec)
   {
      FramePose currentHandPoseInWorld = new FramePose();
      currentHandPoseInWorld.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      currentHandPoseInWorld.changeFrame(world);

      setInput(robotSide, currentHandPoseInWorld, axisOrientationInGraspedObjectFrame, graspedObjectTransformToWorld, totalRotationInRadians,
            rotationRateRadPerSec);
   }

   public void setInput(RobotSide robotSide, FramePose currentHandPoseInWorld, Axis axisOrientationInGraspedObjectFrame,
         RigidBodyTransform graspedObjectTransformToWorld, double totalRotationInRadians, double rotationRateRadPerSec)
   {
      PoseReferenceFrame graspedObjectFrame = new PoseReferenceFrame("graspedObjectFrameBeforeRotation", world);
      graspedObjectFrame.setPoseAndUpdate(graspedObjectTransformToWorld);

      radiansToRotateBetweenHandPoses = Math.abs(radiansToRotateBetweenHandPoses) * Math.signum(totalRotationInRadians);

      int numberOfDiscreteHandPosesToUse = (int) Math.round(totalRotationInRadians / radiansToRotateBetweenHandPoses);

      double totalTrajectoryTime = Math.abs(totalRotationInRadians / rotationRateRadPerSec);
      double trajectoryTimePerHandPose = totalTrajectoryTime / numberOfDiscreteHandPosesToUse;

      if (DEBUG)
      {
         PrintTools.debug(this, "Current Hand Pose in World: " + currentHandPoseInWorld);
         PrintTools.debug(this, "Grasped Object Transform To World: " + graspedObjectTransformToWorld);
         PrintTools.debug(this, "Grasped Object Reference Frame: " + graspedObjectFrame);
         PrintTools.debug(this, "Total Desired Rotation In Radians: " + totalRotationInRadians);
         PrintTools.debug(this, "Number Of Discrete Hand Poses To Use: " + numberOfDiscreteHandPosesToUse);
         PrintTools.debug(this, "Trajectory Time Per Hand Pose: " + trajectoryTimePerHandPose);
      }

      ArrayList<FramePose> handPosesTangentToSemiCircularPath = copyHandPoseAndRotateAboutAxisRecursively(currentHandPoseInWorld, graspedObjectFrame,
            axisOrientationInGraspedObjectFrame, numberOfDiscreteHandPosesToUse);
      
      Point3d[] desiredPositions = new Point3d[numberOfDiscreteHandPosesToUse];
      Quat4d[] desiredOrientations = new Quat4d[numberOfDiscreteHandPosesToUse];

      int i = 0;
      taskExecutor.clear();
            for (FramePose desiredHandPose : handPosesTangentToSemiCircularPath)
            {
               Frame packetFrame;
               if (desiredHandPose.getReferenceFrame().isWorldFrame())
               {
                  packetFrame = Frame.WORLD;
               }
               else
               {
                  throw new RuntimeException("Hand Pose must be defined in World reference frame.");
               }
               if (USE_WHOLE_BODY_IK)
               {
                  taskExecutor.submit(new WholeBodyInverseKinematicTask(robotSide, yoTime, wholeBodyInverseKinematicBehavior, desiredHandPose,
                        trajectoryTimePerHandPose, 0, ControlledDoF.DOF_3P2R, true));
               }
               else
               {
                  desiredPositions[i] = new Point3d();
                  desiredOrientations[i] = new Quat4d();
                  
                  desiredHandPose.getPose(desiredPositions[i], desiredOrientations[i]);
                  i++;
               }
            }

      taskExecutor.submit(new HandPoseListTask(new HandPoseListPacket(robotSide, Frame.WORLD, DataType.HAND_POSE, desiredPositions, desiredOrientations, false,
            totalTrajectoryTime, null), handPoseListBehavior, yoTime));

      hasInputBeenSet.set(true);
   }

   private ArrayList<FramePose> copyHandPoseAndRotateAboutAxisRecursively(FramePose handPoseInWorld, ReferenceFrame graspedObjectFrame,
         Axis axisOrientationInGraspedObjectFrame, int numberOfHandPoses)
   {
      ArrayList<FramePose> desiredHandPoses = new ArrayList<FramePose>(numberOfHandPoses);

      //      desiredHandPoses.add(copyHandPoseAndRotateAboutAxis(handPoseInWorld));
      desiredHandPoses.add(handPoseInWorld.getRotatedAboutAxisCopy(graspedObjectFrame, axisOrientationInGraspedObjectFrame, radiansToRotateBetweenHandPoses));

      for (int i = desiredHandPoses.size(); i < numberOfHandPoses; i++)
      {
         FramePose previousDesiredHandPose = desiredHandPoses.get(i - 1);
         //         FramePose nextDesiredHandPose = copyHandPoseAndRotateAboutAxis(previousDesiredHandPose);
         FramePose nextDesiredHandPose = previousDesiredHandPose.getRotatedAboutAxisCopy(graspedObjectFrame, axisOrientationInGraspedObjectFrame,
               radiansToRotateBetweenHandPoses);
         PrintTools.debug(this, "Adding Desired Hand Pose Position: " + nextDesiredHandPose.printOutPosition());
         desiredHandPoses.add(i, nextDesiredHandPose);
      }
      return desiredHandPoses;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseListBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseListBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      handPoseListBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseListBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      handPoseListBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      handPoseListBehavior.resume();
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
      handPoseListBehavior.finalize();
   }

   @Override
   public void initialize()
   {
      handPoseListBehavior.initialize();
      hasInputBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
