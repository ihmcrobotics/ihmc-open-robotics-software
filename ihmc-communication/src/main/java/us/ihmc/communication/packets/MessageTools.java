package us.ihmc.communication.packets;

import builtin_interfaces.Time;
import controller_msgs.ControllerCrashNotificationPacket;
import controller_msgs.InvalidPacketNotificationPacket;
import controller_msgs.RigidBodyTransformMessage;
import controller_msgs.RobotConfigurationData;
import geometry_msgs.Transform;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import ihmc_common_msgs.Box3DMessage;
import ihmc_common_msgs.Capsule3DMessage;
import ihmc_common_msgs.ConvexPolytope3DMessage;
import ihmc_common_msgs.Cylinder3DMessage;
import ihmc_common_msgs.Ellipsoid3DMessage;
import ihmc_common_msgs.GuidMessage;
import ihmc_common_msgs.InstantMessage;
import ihmc_common_msgs.PoseListMessage;
import ihmc_common_msgs.Ramp3DMessage;
import ihmc_common_msgs.SE3TrajectoryPointMessage;
import ihmc_common_msgs.SelectionMatrix3DMessage;
import ihmc_common_msgs.TextToSpeechPacket;
import ihmc_common_msgs.TrajectoryPoint1DMessage;
import ihmc_common_msgs.UUIDMessage;
import ihmc_common_msgs.WeightMatrix3DMessage;
import ihmc_common_msgs.YoRegistryMessage;
import org.apache.logging.log4j.Level;
import perception_msgs.ImageMessage;
import std_msgs.Bool;
import toolbox_msgs.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import toolbox_msgs.KinematicsToolboxPrivilegedConfigurationMessage;
import toolbox_msgs.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.ToolboxStateMessage;
import toolbox_msgs.WalkingControllerPreviewOutputMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.QuaternionCalculus;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage;
import us.ihmc.euclid.jros2.messages.EuclidPose3DMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.jros2.Guid;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.UUID;

public class MessageTools
{
   public static final boolean DEBUG = false;
   public static final int WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES = 250;

   public static TextToSpeechPacket createTextToSpeechPacket(String textToSpeak)
   {
      if (DEBUG)
         System.out.println("created new TextToSpeechPacket " + textToSpeak);
      TextToSpeechPacket message = new TextToSpeechPacket();
      message.setTextToSpeak(textToSpeak);
      return message;
   }

   public static InvalidPacketNotificationPacket createInvalidPacketNotificationPacket(Class<?> packetClass, String errorMessage)
   {
      InvalidPacketNotificationPacket message = new InvalidPacketNotificationPacket();
      message.setPacketClassSimpleName(packetClass.getSimpleName());
      message.setErrorMessage(errorMessage);
      return message;
   }

   /**
    * Creates a new center of mass message.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details. For
    * example, the priority of the task can be changed by changing the weight of this message, a custom
    * control frame can be specified.
    * </p>
    *
    * @param desiredPosition the position that center of mass should reach. The data is assumed to be
    *                        expressed in world frame. Not modified.
    */
   public static KinematicsToolboxCenterOfMassMessage createKinematicsToolboxCenterOfMassMessage(Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      message.getDesiredPositionInWorld().set(desiredPosition);
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * Before the message can be sent to the solver, you will need to provide at least a desired
    * orientation and/or desired position.
    * </p>
    *
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyReadOnly endEffector)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details. For
    * example, the priority of the task can be changed by changing the weight of this message, a custom
    * control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for linear control only.
    * </p>
    *
    * @param endEffector     the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin should
    *                        reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyReadOnly endEffector, Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredPositionInWorld().set(desiredPosition);
      packSelectionMatrix3DMessage(false, message.getAngularSelectionMatrix());
      packSelectionMatrix3DMessage(true, message.getLinearSelectionMatrix());
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details. For
    * example, the priority of the task can be changed by changing the weight of this message, a custom
    * control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for angular control only.
    * </p>
    *
    * @param endEffector        the end-effector to solver for in the
    *                           {@code KinematicsToolboxController}.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *                           reach. The data is assumed to be expressed in world frame. Not
    *                           modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyReadOnly endEffector, QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredOrientationInWorld().set(desiredOrientation);
      packSelectionMatrix3DMessage(true, message.getAngularSelectionMatrix());
      packSelectionMatrix3DMessage(false, message.getLinearSelectionMatrix());
      return message;
   }

   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyReadOnly endEffector, Pose3DReadOnly desiredPose)
   {
      return createKinematicsToolboxRigidBodyMessage(endEffector, desiredPose.getPosition(), desiredPose.getOrientation());
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details. For
    * example, the priority of the task can be changed by changing the weight of this message, a custom
    * control frame can be specified.
    * </p>
    *
    * @param endEffector        the end-effector to solver for in the
    *                           {@code KinematicsToolboxController}.
    * @param desiredPosition    the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *                           should reach. The data is assumed to be expressed in world frame. Not
    *                           modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *                           reach. The data is assumed to be expressed in world frame. Not
    *                           modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyReadOnly endEffector,
                                                                                           Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      if (desiredPosition == null)
         return createKinematicsToolboxRigidBodyMessage(endEffector, desiredOrientation);
      else if (desiredOrientation == null)
         return createKinematicsToolboxRigidBodyMessage(endEffector, desiredPosition);

      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredPositionInWorld().set(desiredPosition);
      message.getDesiredOrientationInWorld().set(desiredOrientation);
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details. For
    * example, the priority of the task can be changed by changing the weight of this message, a custom
    * control frame can be specified.
    * </p>
    *
    * @param endEffector        the end-effector to solver for in the
    *                           {@code KinematicsToolboxController}.
    * @param controlFrame       specifies the location and orientation of interest for controlling the
    *                           end-effector.
    * @param desiredPosition    the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *                           should reach. The data is assumed to be expressed in world frame. Not
    *                           modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *                           reach. The data is assumed to be expressed in world frame. Not
    *                           modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyReadOnly endEffector,
                                                                                           ReferenceFrame controlFrame,
                                                                                           Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredPositionInWorld().set(desiredPosition);
      message.getDesiredOrientationInWorld().set(desiredOrientation);
      RigidBodyTransform transformToBodyFixedFrame = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(transformToBodyFixedFrame, endEffector.getBodyFixedFrame());
      message.getControlFramePositionInEndEffector().set(transformToBodyFixedFrame.getTranslation());
      message.getControlFrameOrientationInEndEffector().set(transformToBodyFixedFrame.getRotation());
      return message;
   }

   public static SelectionMatrix3DMessage createSelectionMatrix3DMessage(boolean selected)
   {
      return createSelectionMatrix3DMessage(selected, selected, selected);
   }

   public static SelectionMatrix3DMessage createSelectionMatrix3DMessage(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      return createSelectionMatrix3DMessage(xSelected, ySelected, zSelected, null);
   }

   public static SelectionMatrix3DMessage createSelectionMatrix3DMessage(boolean xSelected, boolean ySelected, boolean zSelected, ReferenceFrame selectionFrame)
   {
      SelectionMatrix3DMessage message = new SelectionMatrix3DMessage();
      message.setXSelected(xSelected);
      message.setYSelected(ySelected);
      message.setZSelected(zSelected);
      message.setSelectionFrameId(toFrameId(selectionFrame));
      return message;
   }

   /**
    * Copy constructor.
    *
    * @param selectionMatrix3D the original selection matrix to copy. Not modified.
    */
   public static SelectionMatrix3DMessage createSelectionMatrix3DMessage(SelectionMatrix3D selectionMatrix3D)
   {
      SelectionMatrix3DMessage message = new SelectionMatrix3DMessage();
      packSelectionMatrix3DMessage(selectionMatrix3D, message);
      return message;
   }

   public static void packSelectionMatrix3DMessage(SelectionMatrix3D selectionMatrix3D, SelectionMatrix3DMessage messageToPack)
   {
      messageToPack.setSelectionFrameId(MessageTools.toFrameId(selectionMatrix3D.getSelectionFrame()));
      messageToPack.setXSelected(selectionMatrix3D.isXSelected());
      messageToPack.setYSelected(selectionMatrix3D.isYSelected());
      messageToPack.setZSelected(selectionMatrix3D.isZSelected());
   }

   public static void packSelectionMatrix3DMessage(boolean xSelected,
                                                   boolean ySelected,
                                                   boolean zSelected,
                                                   ReferenceFrame selectionFrame,
                                                   SelectionMatrix3DMessage messageToPack)
   {
      messageToPack.setSelectionFrameId(toFrameId(selectionFrame));
      messageToPack.setXSelected(xSelected);
      messageToPack.setYSelected(ySelected);
      messageToPack.setZSelected(zSelected);
   }

   public static void packSelectionMatrix3DMessage(boolean selected, SelectionMatrix3DMessage messageToPack)
   {
      packSelectionMatrix3DMessage(selected, selected, selected, null, messageToPack);
   }

   public static WeightMatrix3DMessage createWeightMatrix3DMessage(WeightMatrix3D weightMatrix)
   {
      WeightMatrix3DMessage message = new WeightMatrix3DMessage();
      message.setWeightFrameId(MessageTools.toFrameId(weightMatrix.getWeightFrame()));
      message.setXWeight(weightMatrix.getXAxisWeight());
      message.setYWeight(weightMatrix.getYAxisWeight());
      message.setZWeight(weightMatrix.getZAxisWeight());
      return message;
   }

   public static WeightMatrix3DMessage createWeightMatrix3DMessage(double weight)
   {
      WeightMatrix3DMessage message = new WeightMatrix3DMessage();
      packWeightMatrix3DMessage(weight, message);
      return message;
   }

   public static void packWeightMatrix3DMessage(Tuple3DReadOnly weight, WeightMatrix3DMessage messageToPack)
   {
      messageToPack.setWeightFrameId(MessageTools.toFrameId(null));
      messageToPack.setXWeight(weight.getX());
      messageToPack.setYWeight(weight.getY());
      messageToPack.setZWeight(weight.getZ());
   }

   public static void packWeightMatrix3DMessage(double weight, WeightMatrix3DMessage messageToPack)
   {
      messageToPack.setWeightFrameId(MessageTools.toFrameId(null));
      messageToPack.setXWeight(weight);
      messageToPack.setYWeight(weight);
      messageToPack.setZWeight(weight);
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(OneDoFJointBasics[] joints)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.setJointNameHash(Arrays.hashCode(joints));
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(FloatingJointBasics rootJoint, OneDoFJointBasics[] newJointData)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.setJointNameHash(Arrays.hashCode(newJointData));
      MessageTools.packDesiredJointState(message, rootJoint, newJointData);
      return message;
   }

   public static WalkingControllerPreviewOutputMessage createWalkingControllerPreviewOutputMessage(double inputDT,
                                                                                                   List<KinematicsToolboxOutputStatus> previewFrames)
   {
      WalkingControllerPreviewOutputMessage message = new WalkingControllerPreviewOutputMessage();

      if (previewFrames.size() <= WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES)
      {
         message.setFrameDt(inputDT);
         for (KinematicsToolboxOutputStatus frame : previewFrames)
         {
            message.getRobotConfigurations().add().set(frame);
         }
      }
      else
      {
         double outputDT = inputDT * (double) previewFrames.size() / (double) WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES;
         message.setFrameDt(outputDT);

         for (int outputFrameIndex = 0; outputFrameIndex < WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES; outputFrameIndex++)
         {
            double outputFrameTime = outputFrameIndex * outputDT;
            int firstInputFrameIndex = (int) Math.floor(outputFrameTime / inputDT);
            int secondInputFrameIndex = (int) Math.ceil(outputFrameTime / inputDT);
            if (firstInputFrameIndex == secondInputFrameIndex)
            {
               message.getRobotConfigurations().add().set(previewFrames.get(firstInputFrameIndex));
            }
            else
            {
               double firstInputFrameTime = firstInputFrameIndex * inputDT;
               double secondInputFrameTime = secondInputFrameIndex * inputDT;
               double alpha = (secondInputFrameTime - outputFrameTime) / (secondInputFrameTime - firstInputFrameTime);
               message.getRobotConfigurations()
                      .add()
                      .set(interpolateMessages(previewFrames.get(firstInputFrameIndex), previewFrames.get(secondInputFrameIndex), alpha));
            }
         }
      }
      return message;
   }

   public static ControllerCrashNotificationPacket createControllerCrashNotificationPacket(Throwable exception)
   {
      return createControllerCrashNotificationPacket(null, exception);
   }

   public static ControllerCrashNotificationPacket createControllerCrashNotificationPacket(ControllerCrashLocation location, Throwable exception)
   {
      ControllerCrashNotificationPacket message = new ControllerCrashNotificationPacket();
      message.setControllerCrashLocation(location != null ? location.toByte() : -1);
      message.setExceptionType(exception.getClass().getSimpleName());
      message.setErrorMessage(exception.getMessage());

      StackTraceElement[] stackTrace = exception.getStackTrace();

      message.getStacktrace().clear();

      if (stackTrace != null)
      {
         int length = Math.min(50, stackTrace.length);

         for (int i = 0; i < length; i++)
         {
            message.getStacktrace().add(stackTrace[i].toString());
         }
      }

      return message;
   }

   public static ToolboxStateMessage createToolboxStateMessage(ToolboxState requestedState)
   {
      ToolboxStateMessage message = new ToolboxStateMessage();
      message.setRequestedToolboxState(requestedState.toByte());
      return message;
   }

   public static <T extends Enum<T>> T fromByteToEnum(byte value, Class<T> enumType)
   {
      return enumType.getEnumConstants()[(int) value];
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TByteArrayList#reset()} on {@code destination}.
    *
    * @param source      the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TByteArrayList source, TByteArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TDoubleArrayList#reset()} on {@code destination}.
    *
    * @param source      the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TDoubleArrayList source, TDoubleArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TFloatArrayList#reset()} on {@code destination}.
    *
    * @param source      the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TFloatArrayList source, TFloatArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TIntArrayList#reset()} on {@code destination}.
    *
    * @param source      the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TIntArrayList source, TIntArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TLongArrayList#reset()} on {@code destination}.
    *
    * @param source      the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(TLongArrayList source, TLongArrayList destination)
   {
      destination.reset();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add(source.getQuick(i));
      }
   }

   public static <T extends ROS2Message<T>> void copyData(T[] source, IDLObjectSequence<T> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (T item : source)
         destination.add().set(item);
   }

   public static <T extends ROS2Message<T>> void copyData(List<T> source, IDLObjectSequence<T> destination)
   {
      destination.clear();

      if (source == null || source.isEmpty())
         return;

      for (T item : source)
         destination.add().set(item);
   }

   public static void copyData(Pose3DReadOnly[] source, IDLObjectSequence<EuclidPose3DMessage> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (Pose3DReadOnly pose : source)
         destination.add().set(pose);
   }

   public static void copyData(Point3DReadOnly[] source, IDLObjectSequence<EuclidPoint3DMessage> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (Point3DReadOnly point : source)
         destination.add().set(point);
   }

   public static void copyData(Iterable<? extends Point3DReadOnly> source, IDLObjectSequence<EuclidPoint3DMessage> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (Point3DReadOnly point : source)
         destination.add().set(point);
   }

   /**
    * Performs a deep copy of the data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    *
    * @param source      the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    * @param <T>         Should be either {@code Enum}, {@code StringBuilder}, or {@code Settable<T>}.
    * @throws IllegalArgumentException if the type {@code T} is none of the following: {@code Enum},
    *       {@code StringBuilder}, {@code Settable<T>}.
    */
   @SuppressWarnings("unchecked")
   public static <T> void copyData(List<T> source, RecyclingArrayList<T> destination)
   {
      destination.clear();

      if (source == null || source.isEmpty())
         return;

      T firstElement = destination.add();

      if (firstElement instanceof Settable)
      {
         destination.clear();

         for (int i = 0; i < source.size(); i++)
         {
            ((Settable<T>) destination.add()).set(source.get(i));
         }
      }
      else if (firstElement instanceof StringBuilder)
      {
         destination.clear();

         for (int i = 0; i < source.size(); i++)
         {
            StringBuilder destinationElement = (StringBuilder) destination.add();
            destinationElement.setLength(0);
            destinationElement.append((StringBuilder) source.get(i));
         }
      }
      else
      {
         throw new IllegalArgumentException(
               MessageTools.class.getSimpleName() + ".copyData(...) can only be used with " + RecyclingArrayList.class.getSimpleName()
               + "s declared with either of the following types: Enum, StringBuilder, and" + Settable.class.getSimpleName());
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    *
    * @param source      the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static <T extends ROS2Message<T>> void copyData(T[] source, RecyclingArrayList<T> destination)
   {
      destination.clear();

      if (source == null)
         return;

      try
      {
         for (int i = 0; i < source.length; i++)
         {
            destination.add().set(source[i]);
         }
      }
      catch (ArrayIndexOutOfBoundsException e)
      {
         LogTools.error("Caught exception while copying data from array of length: " + source.length);
         throw e;
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    *
    * @param source      the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(String[] source, RecyclingArrayList<StringBuilder> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.length; i++)
      {
         StringBuilder destinationElement = destination.add();
         destinationElement.setLength(0);
         destinationElement.append(source[i]);
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    *
    * @param source      the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(StringBuilder[] source, RecyclingArrayList<StringBuilder> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.length; i++)
      {
         StringBuilder destinationElement = destination.add();
         destinationElement.setLength(0);
         destinationElement.append(source[i]);
      }
   }

   public static <T> List<T> toList(RecyclingArrayList<T> original)
   {
      List<T> list = new ArrayList<>();
      for (int i = 0; i < original.size(); i++)
         list.add(original.get(i));
      return list;
   }

   public static <T extends EpsilonComparable<T>> boolean epsilonEquals(RecyclingArrayList<T> listOne, RecyclingArrayList<T> listTwo, double epsilon)
   {
      if (listOne.size() != listTwo.size())
         return false;
      for (int i = 0; i < listOne.size(); i++)
      {
         if (!listOne.get(i).epsilonEquals(listTwo.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static boolean epsilonEquals(TDoubleArrayList listOne, TDoubleArrayList listTwo, double epsilon)
   {
      if (listOne.size() != listTwo.size())
         return false;
      for (int i = 0; i < listOne.size(); i++)
      {
         if (!MathTools.epsilonEquals(listOne.get(i), listTwo.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static boolean epsilonEquals(TFloatArrayList listOne, TFloatArrayList listTwo, double epsilon)
   {
      if (listOne.size() != listTwo.size())
         return false;
      for (int i = 0; i < listOne.size(); i++)
      {
         if (!MathTools.epsilonEquals(listOne.get(i), listTwo.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static long toFrameId(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
         return EuclidHashCodeTools.NULL_HASHCODE;
      else
         return referenceFrame.getFrameNameHashCode();
   }

   /**
    * Reads desired configuration and velocity from {@code kinematicsToolboxOutputStatus} and updates
    * the state of the given joints.
    * <p>
    * Note that for {@code rootJoint} both the linear and angular velocities are assumed to be
    * expressed in the joint's local coordinate system.
    * </p>
    *
    * @param kinematicsToolboxOutputStatus the message to get data from. Not modified.
    * @param rootJointToUpdate             the floating to update configuration & velocity of.
    *                                      Modified.
    * @param jointsToUpdate                the 1-DoF joints to update configuration & velocity of.
    *                                      Modified.
    */
   public static void unpackDesiredJointState(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus,
                                              FloatingJointBasics rootJointToUpdate,
                                              OneDoFJointBasics[] jointsToUpdate)
   {
      if (kinematicsToolboxOutputStatus.getDesiredJointAngles().isEmpty())
         return;

      int jointNameHash = Arrays.hashCode(jointsToUpdate);

      if (jointNameHash != kinematicsToolboxOutputStatus.getJointNameHash())
         throw new RuntimeException("The robots are different.");

      for (int i = 0; i < kinematicsToolboxOutputStatus.getDesiredJointAngles().size(); i++)
         jointsToUpdate[i].setQ(kinematicsToolboxOutputStatus.getDesiredJointAngles().get(i));
      for (int i = 0; i < kinematicsToolboxOutputStatus.getDesiredJointVelocities().size(); i++)
         jointsToUpdate[i].setQd(kinematicsToolboxOutputStatus.getDesiredJointVelocities().get(i));

      Point3D desiredRootPosition = kinematicsToolboxOutputStatus.getDesiredRootPosition().getPoint();
      Quaternion desiredRootOrientation = kinematicsToolboxOutputStatus.getDesiredRootOrientation().getQuaternion();
      Vector3D desiredRootLinearVelocity = kinematicsToolboxOutputStatus.getDesiredRootLinearVelocity().getVector();
      Vector3D desiredRootAngularVelocity = kinematicsToolboxOutputStatus.getDesiredRootAngularVelocity().getVector();
      rootJointToUpdate.getJointPose().set(desiredRootPosition, desiredRootOrientation);
      rootJointToUpdate.getJointTwist().set(desiredRootAngularVelocity, desiredRootLinearVelocity);
   }

   /**
    * Packs the configuration and velocity from {@code rootJoint} and each joint in
    * {@code newJonitData} into {@code kinematicsToolboxOutputStatusToPack}.
    * <p>
    * Note that for {@code rootJoint} both the linear and angular velocities are expressed in the
    * joint's local coordinate system.
    * </p>
    *
    * @param kinematicsToolboxOutputStatusToPack the message in which the desired joint state
    *                                            (configuration & velocity) is to be sorted. Modified.
    * @param rootJoint                           the floating joint to get data from. Not modified.
    * @param newJointData                        the 1-DoF joints to get data from. Not modified.
    */
   public static void packDesiredJointState(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatusToPack,
                                            FloatingJointReadOnly rootJoint,
                                            OneDoFJointReadOnly[] newJointData)
   {
      int jointNameHash = Arrays.hashCode(newJointData);

      if (jointNameHash != kinematicsToolboxOutputStatusToPack.getJointNameHash())
         throw new RuntimeException("The robots are different.");

      kinematicsToolboxOutputStatusToPack.getDesiredJointAngles().clear();
      kinematicsToolboxOutputStatusToPack.getDesiredJointVelocities().clear();

      for (int i = 0; i < newJointData.length; i++)
      {
         OneDoFJointReadOnly joint = newJointData[i];
         kinematicsToolboxOutputStatusToPack.getDesiredJointAngles().add((float) joint.getQ());
         kinematicsToolboxOutputStatusToPack.getDesiredJointVelocities().add((float) joint.getQd());
      }

      Point3D desiredRootTranslation = kinematicsToolboxOutputStatusToPack.getDesiredRootPosition().getPoint();
      Quaternion desiredRootOrientation = kinematicsToolboxOutputStatusToPack.getDesiredRootOrientation().getQuaternion();
      Vector3D desiredRootLinearVelocity = kinematicsToolboxOutputStatusToPack.getDesiredRootLinearVelocity().getVector();
      Vector3D desiredRootAngularVelocity = kinematicsToolboxOutputStatusToPack.getDesiredRootAngularVelocity().getVector();

      if (rootJoint != null)
      {
         Pose3DReadOnly jointPose = rootJoint.getJointPose();
         TwistReadOnly jointTwist = rootJoint.getJointTwist();

         desiredRootTranslation.set(jointPose.getPosition());
         desiredRootOrientation.set(jointPose.getOrientation());
         desiredRootLinearVelocity.set(jointTwist.getLinearPart());
         desiredRootAngularVelocity.set(jointTwist.getAngularPart());
      }
      else
      {
         desiredRootTranslation.setToZero();
         desiredRootOrientation.setToZero();
         desiredRootLinearVelocity.setToZero();
         desiredRootAngularVelocity.setToZero();
      }
   }

   public static KinematicsToolboxOutputStatus interpolateMessages(KinematicsToolboxOutputStatus outputStatusOne,
                                                                   KinematicsToolboxOutputStatus outputStatusTwo,
                                                                   double alpha)
   {
      KinematicsToolboxOutputStatus interpolated = new KinematicsToolboxOutputStatus();
      interpolateMessages(outputStatusOne, outputStatusTwo, alpha, interpolated);
      return interpolated;
   }

   public static void interpolateMessages(KinematicsToolboxOutputStatus outputStatusOne,
                                          KinematicsToolboxOutputStatus outputStatusTwo,
                                          double alpha,
                                          KinematicsToolboxOutputStatus interpolatedToPack)
   {
      if (outputStatusOne.getJointNameHash() != outputStatusTwo.getJointNameHash())
         throw new RuntimeException("Output status are not compatible.");

      interpolatedToPack.getDesiredJointAngles().clear();
      interpolatedToPack.getDesiredJointVelocities().clear();

      var jointAngles1 = outputStatusOne.getDesiredJointAngles();
      var jointAngles2 = outputStatusTwo.getDesiredJointAngles();
      var jointVelocities1 = outputStatusOne.getDesiredJointVelocities();
      var jointVelocities2 = outputStatusTwo.getDesiredJointVelocities();

      for (int i = 0; i < jointAngles1.size(); i++)
      {
         interpolatedToPack.getDesiredJointAngles().add((float) EuclidCoreTools.interpolate(jointAngles1.get(i), jointAngles2.get(i), alpha));
         interpolatedToPack.getDesiredJointVelocities().add((float) EuclidCoreTools.interpolate(jointVelocities1.get(i), jointVelocities2.get(i), alpha));
      }

      Point3D rootPosition1 = outputStatusOne.getDesiredRootPosition().getPoint();
      Point3D rootPosition2 = outputStatusTwo.getDesiredRootPosition().getPoint();
      Quaternion rootOrientation1 = outputStatusOne.getDesiredRootOrientation().getQuaternion();
      Quaternion rootOrientation2 = outputStatusTwo.getDesiredRootOrientation().getQuaternion();
      Vector3D rootLinearVelocity1 = outputStatusOne.getDesiredRootLinearVelocity().getVector();
      Vector3D rootLinearVelocity2 = outputStatusTwo.getDesiredRootLinearVelocity().getVector();
      Vector3D rootAngularVelocity1 = outputStatusOne.getDesiredRootAngularVelocity().getVector();
      Vector3D rootAngularVelocity2 = outputStatusTwo.getDesiredRootAngularVelocity().getVector();

      interpolatedToPack.getDesiredRootPosition().getPoint().interpolate(rootPosition1, rootPosition2, alpha);
      interpolatedToPack.getDesiredRootOrientation().getQuaternion().interpolate(rootOrientation1, rootOrientation2, alpha);
      interpolatedToPack.getDesiredRootLinearVelocity().getVector().interpolate(rootLinearVelocity1, rootLinearVelocity2, alpha);
      interpolatedToPack.getDesiredRootAngularVelocity().getVector().interpolate(rootAngularVelocity1, rootAngularVelocity2, alpha);

      interpolatedToPack.setJointNameHash(outputStatusOne.getJointNameHash());
   }

   /**
    * Interpolates from {@code start} to {@code end} given {@code alpha} &in;[0,1].
    *
    * @param start    the value when {@code alpha = 0}. Not modified.
    * @param end      the value when {@code alpha = 1}. Not modified.
    * @param alpha    the interpolation variable.
    * @param alphaDot the time-derivative of {@code alpha}.
    * @return the result of the interpolation.
    */
   public static KinematicsToolboxOutputStatus interpolate(KinematicsToolboxOutputStatus start,
                                                           KinematicsToolboxOutputStatus end,
                                                           double alpha,
                                                           double alphaDot)
   {
      KinematicsToolboxOutputStatus interpolated = new KinematicsToolboxOutputStatus();
      interpolate(start, end, alpha, alphaDot, interpolated);
      return interpolated;
   }

   /**
    * Interpolates from {@code start} to {@code end} given {@code alpha} &in;[0,1].
    *
    * @param start              the value when {@code alpha = 0}. Not modified.
    * @param end                the value when {@code alpha = 1}. Not modified.
    * @param alpha              the interpolation variable.
    * @param alphaDot           the time-derivative of {@code alpha}.
    * @param interpolatedToPack the message used to store the result of the interpolation. Modified.
    */
   public static void interpolate(KinematicsToolboxOutputStatus start,
                                  KinematicsToolboxOutputStatus end,
                                  double alpha,
                                  double alphaDot,
                                  KinematicsToolboxOutputStatus interpolatedToPack)
   {
      if (start.getJointNameHash() != end.getJointNameHash())
         throw new IllegalArgumentException("start and end are not compatible");

      interpolatedToPack.setJointNameHash(start.getJointNameHash());

      // 1-DoF joints:
      var jointAnglesStart = start.getDesiredJointAngles();
      var jointAnglesEnd = end.getDesiredJointAngles();
      var jointAnglesInterpolated = interpolatedToPack.getDesiredJointAngles();
      var jointVelocitiesStart = start.getDesiredJointVelocities();
      var jointVelocitiesEnd = end.getDesiredJointVelocities();
      var jointVelocitiesInterpolated = interpolatedToPack.getDesiredJointVelocities();

      if (jointAnglesStart.size() != jointAnglesEnd.size() || jointVelocitiesStart.size() != jointVelocitiesEnd.size())
         throw new IllegalArgumentException("start and end are not compatible");

      jointAnglesInterpolated.clear();
      jointVelocitiesInterpolated.clear();

      for (int i = 0; i < jointAnglesStart.size(); i++)
      {
         float q = (float) EuclidCoreTools.interpolate(jointAnglesStart.get(i), jointAnglesEnd.get(i), alpha);
         jointAnglesInterpolated.add(q);
      }

      for (int i = 0; i < jointVelocitiesStart.size(); i++)
      {
         double qDot = alphaDot * (jointAnglesEnd.get(i) - jointAnglesStart.get(i));
         qDot += EuclidCoreTools.interpolate(jointVelocitiesStart.get(i), jointVelocitiesEnd.get(i), alpha);
         jointVelocitiesInterpolated.add((float) qDot);
      }

      // Root joint:
      Quaternion orientationStart = start.getDesiredRootOrientation().getQuaternion();
      Quaternion orientationEnd = end.getDesiredRootOrientation().getQuaternion();
      Quaternion orientationInterpolated = interpolatedToPack.getDesiredRootOrientation().getQuaternion();
      Point3D positionStart = start.getDesiredRootPosition().getPoint();
      Point3D positionEnd = end.getDesiredRootPosition().getPoint();
      Point3D positionInterpolated = interpolatedToPack.getDesiredRootPosition().getPoint();

      Vector3D angularVelocityStart = start.getDesiredRootAngularVelocity().getVector();
      Vector3D angularVelocityEnd = end.getDesiredRootAngularVelocity().getVector();
      Vector3D angularVelocityInterpolated = interpolatedToPack.getDesiredRootAngularVelocity().getVector();
      Vector3D linearVelocityEnd = end.getDesiredRootLinearVelocity().getVector();
      Vector3D linearVelocityStart = start.getDesiredRootLinearVelocity().getVector();
      Vector3D linearVelocityInterpolated = interpolatedToPack.getDesiredRootLinearVelocity().getVector();

      // Do configuration
      orientationInterpolated.interpolate(orientationStart, orientationEnd, alpha);
      positionInterpolated.interpolate(positionStart, positionEnd, alpha);

      // Root joint velocity
      Vector4D quaternionDot = new Vector4D();
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      quaternionDot.sub(orientationEnd, orientationStart);
      quaternionDot.scale(alphaDot);
      quaternionCalculus.computeAngularVelocityInRotatedFrame(orientationInterpolated, quaternionDot, angularVelocityInterpolated);
      angularVelocityInterpolated.scaleAdd(1.0 - alpha, angularVelocityStart, angularVelocityInterpolated);
      angularVelocityInterpolated.scaleAdd(alpha, angularVelocityEnd, angularVelocityInterpolated);

      linearVelocityInterpolated.sub(positionEnd, positionStart);
      linearVelocityInterpolated.scale(alphaDot);
      orientationInterpolated.inverseTransform(linearVelocityInterpolated);
      linearVelocityInterpolated.scaleAdd(1.0 - alpha, linearVelocityStart, linearVelocityInterpolated);
      linearVelocityInterpolated.scaleAdd(alpha, linearVelocityEnd, linearVelocityInterpolated);
   }

   /**
    * Provides a privileged configuration that the {@code KinematicsToolboxController} will use as a
    * reference and attempt to find the solution that is the closest.
    * <p>
    * Avoid calling this method directly, use instead the {@code KinematicsToolboxInputHelper}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore preferable
    * to send the privileged configuration as soon as possible.
    * </p>
    *
    * @param rootJointPosition    the privileged root joint position. Not modified.
    * @param rootJointOrientation the privileged root joint orientation. Not modified.
    * @param jointHashCodes       allows to safely identify to which joint each angle in
    *                             {@link #privilegedJointAngles} belongs to. The hash code can be
    *                             obtained from {@link OneDoFJointBasics#hashCode()}. Not modified.
    * @param jointAngles          the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and {@code jointHashCodes}
    *       are different.
    */
   public static void packPrivilegedRobotConfiguration(KinematicsToolboxPrivilegedConfigurationMessage message,
                                                       Tuple3DReadOnly rootJointPosition,
                                                       QuaternionReadOnly rootJointOrientation,
                                                       int[] jointHashCodes,
                                                       float[] jointAngles)
   {
      message.getPrivilegedRootJointPosition().set(rootJointPosition);
      message.getPrivilegedRootJointOrientation().set(rootJointOrientation);
      MessageTools.packPrivilegedJointAngles(message, jointHashCodes, jointAngles);
   }

   /**
    * When provided, the {@code KinematicsToolboxController} will attempt to find the closest solution
    * to the privileged configuration.
    * <p>
    * Avoid calling this method directly, use instead the {@code KinematicsToolboxInputHelper}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore preferable
    * to send the privileged configuration as soon as possible.
    * </p>
    *
    * @param jointHashCodes allows to safely identify to which joint each angle in
    *                       {@link #privilegedJointAngles} belongs to. The hash code can be obtained
    *                       from {@link OneDoFJointBasics#hashCode()}. Not modified.
    * @param jointAngles    the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and {@code jointHashCodes}
    *       are different.
    */
   public static void packPrivilegedJointAngles(KinematicsToolboxPrivilegedConfigurationMessage message, int[] jointHashCodes, float[] jointAngles)
   {
      if (jointHashCodes.length != jointAngles.length)
         throw new IllegalArgumentException("The two arrays jointAngles and jointHashCodes have to be of same length.");

      message.getPrivilegedJointHashCodes().clear();
      message.getPrivilegedJointHashCodes().addAll(jointHashCodes);
      message.getPrivilegedJointAngles().clear();
      message.getPrivilegedJointAngles().addAll(jointAngles);
   }

   public static void packInitialJointAngles(KinematicsStreamingToolboxInitialConfigurationMessage message, int[] jointHashCodes, float[] jointAngles)
   {
      if (jointHashCodes.length != jointAngles.length)
         throw new IllegalArgumentException("The two arrays jointAngles and jointHashCodes have to be of same length.");

      message.getInitialJointHashCodes().clear();
      message.getInitialJointHashCodes().addAll(jointHashCodes);
      message.getInitialJointAngles().clear();
      message.getInitialJointAngles().addAll(jointAngles);
   }

   /*
    * Set the sequence ID for a RobotConfigurationData object and propagate it to all sensor values.
    */
   public static void setRobotConfigurationDataSequenceId(RobotConfigurationData robotConfigurationData, long sequenceId)
   {
      robotConfigurationData.setSequenceId(sequenceId);
      // update the sequence ID for the sensor data as well
      for (int i = 0; i < robotConfigurationData.getImuSensorData().size(); i++)
      {
         robotConfigurationData.getImuSensorData().get(i).setSequenceId(sequenceId);
      }
      for (int i = 0; i < robotConfigurationData.getForceSensorData().size(); i++)
      {
         robotConfigurationData.getForceSensorData().get(i).setSequenceId(sequenceId);
      }
   }

   public static void toMessage(RigidBodyTransform rigidBodyTransform, RigidBodyTransformMessage rigidBodyTransformMessage)
   {
      rigidBodyTransformMessage.setX(rigidBodyTransform.getTranslation().getX());
      rigidBodyTransformMessage.setY(rigidBodyTransform.getTranslation().getY());
      rigidBodyTransformMessage.setZ(rigidBodyTransform.getTranslation().getZ());
      rigidBodyTransformMessage.setM00(rigidBodyTransform.getRotation().getM00());
      rigidBodyTransformMessage.setM01(rigidBodyTransform.getRotation().getM01());
      rigidBodyTransformMessage.setM02(rigidBodyTransform.getRotation().getM02());
      rigidBodyTransformMessage.setM10(rigidBodyTransform.getRotation().getM10());
      rigidBodyTransformMessage.setM11(rigidBodyTransform.getRotation().getM11());
      rigidBodyTransformMessage.setM12(rigidBodyTransform.getRotation().getM12());
      rigidBodyTransformMessage.setM20(rigidBodyTransform.getRotation().getM20());
      rigidBodyTransformMessage.setM21(rigidBodyTransform.getRotation().getM21());
      rigidBodyTransformMessage.setM22(rigidBodyTransform.getRotation().getM22());
   }

   public static void toEuclid(RigidBodyTransformMessage rigidBodyTransformMessage, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getTranslation().setX(rigidBodyTransformMessage.getX());
      rigidBodyTransform.getTranslation().setY(rigidBodyTransformMessage.getY());
      rigidBodyTransform.getTranslation().setZ(rigidBodyTransformMessage.getZ());
      rigidBodyTransform.getRotation()
                        .setUnsafe(rigidBodyTransformMessage.getM00(),
                                   rigidBodyTransformMessage.getM01(),
                                   rigidBodyTransformMessage.getM02(),
                                   rigidBodyTransformMessage.getM10(),
                                   rigidBodyTransformMessage.getM11(),
                                   rigidBodyTransformMessage.getM12(),
                                   rigidBodyTransformMessage.getM20(),
                                   rigidBodyTransformMessage.getM21(),
                                   rigidBodyTransformMessage.getM22());
   }

   public static RigidBodyTransform toEuclid(RigidBodyTransformMessage rigidBodyTransformMessage)
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      toEuclid(rigidBodyTransformMessage, rigidBodyTransform);
      return rigidBodyTransform;
   }

   public static void toMessage(RigidBodyTransformReadOnly rigidBodyTransform, Transform transformMessage)
   {
      transformMessage.getTranslation().setX(rigidBodyTransform.getTranslationX());
      transformMessage.getTranslation().setY(rigidBodyTransform.getTranslationY());
      transformMessage.getTranslation().setZ(rigidBodyTransform.getTranslationZ());
      Quaternion quaternion = new Quaternion();
      quaternion.set(rigidBodyTransform.getRotation());
      transformMessage.getRotation().setX(quaternion.getX());
      transformMessage.getRotation().setY(quaternion.getY());
      transformMessage.getRotation().setZ(quaternion.getZ());
      transformMessage.getRotation().setW(quaternion.getS());
   }

   public static void toEuclid(Transform transformMessage, RigidBodyTransform rigidBodyTransformToPack)
   {
      rigidBodyTransformToPack.getTranslation()
                            .set(transformMessage.getTranslation().getX(),
                                 transformMessage.getTranslation().getY(),
                                 transformMessage.getTranslation().getZ());
      Quaternion quaternion = new Quaternion();
      quaternion.set(transformMessage.getRotation().getX(),
                     transformMessage.getRotation().getY(),
                     transformMessage.getRotation().getZ(),
                     transformMessage.getRotation().getW());
      rigidBodyTransformToPack.getRotation().set(quaternion);
   }

   public static Box3DMessage createBox3DMessage(Box3DReadOnly box)
   {
      Box3DMessage message = new Box3DMessage();
      packBox3DMessage(box, message);
      return message;
   }

   public static ConvexPolytope3DMessage createConvexPolytope3DMessage(ConvexPolytope3DReadOnly polytope)
   {
      ConvexPolytope3DMessage message = new ConvexPolytope3DMessage();
      packConvexPolytope3DMessage(polytope, message);
      return message;
   }

   public static Capsule3DMessage createCapsule3DMessage(Capsule3DReadOnly capsule)
   {
      Capsule3DMessage message = new Capsule3DMessage();
      packCapsule3DMessage(capsule, message);
      return message;
   }

   public static Ellipsoid3DMessage createEllipsoid3DMessage(Ellipsoid3DReadOnly ellipsoid)
   {
      Ellipsoid3DMessage message = new Ellipsoid3DMessage();
      packEllipsoid3DMessage(ellipsoid, message);
      return message;
   }

   public static void packBox3DMessage(Box3DReadOnly box, Box3DMessage boxMessageToSet)
   {
      boxMessageToSet.getSize().set(box.getSize());
      boxMessageToSet.getPose().set(box.getPose());
   }

   public static void packConvexPolytope3DMessage(ConvexPolytope3DReadOnly polytope, ConvexPolytope3DMessage convexPolytopeMessageToSet)
   {
      convexPolytopeMessageToSet.getVertices().clear();

      for (int i = 0; i < polytope.getNumberOfVertices(); i++)
      {
         convexPolytopeMessageToSet.getVertices().add().set(polytope.getVertex(i));
      }
   }

   public static void packCylinder3DMessage(Cylinder3DReadOnly cylinder, Cylinder3DMessage cylinderMessageToSet)
   {
      cylinderMessageToSet.getPosition().set(cylinder.getPosition());
      cylinderMessageToSet.getAxis().set(cylinder.getAxis());
      cylinderMessageToSet.setRadius(cylinder.getRadius());
      cylinderMessageToSet.setLength(cylinder.getLength());
   }

   public static void packCapsule3DMessage(Capsule3DReadOnly capsule, Capsule3DMessage capsuleMessageToSet)
   {
      capsuleMessageToSet.getPosition().set(capsule.getPosition());
      capsuleMessageToSet.getAxis().set(capsule.getAxis());
      capsuleMessageToSet.setRadius(capsule.getRadius());
      capsuleMessageToSet.setLength(capsule.getLength());
   }

   public static void packEllipsoid3DMessage(Ellipsoid3DReadOnly ellipsoid, Ellipsoid3DMessage ellipsoidMessageToSet)
   {
      ellipsoidMessageToSet.getPose().set(ellipsoid.getPose());
      ellipsoidMessageToSet.getRadii().set(ellipsoid.getRadii());
   }

   public static void unpackBox3DMessage(Box3DMessage boxMessage, Box3DBasics boxToSet)
   {
      boxToSet.getSize().set(boxMessage.getSize().getVector());
      boxToSet.getPose().set(boxMessage.getPose().getPose());
   }

   public static void unpackRamp3DMessage(Ramp3DMessage rampMessage, Ramp3DBasics rampToSet)
   {
      rampToSet.getSize().set(rampMessage.getSize().getVector());
      rampToSet.getPose().set(rampMessage.getPose().getPose());
   }

   public static void unpackConvexPolytope3DMessage(ConvexPolytope3DMessage convexPolytopeMessage, FrameConvexPolytope3D polytopeToSet)
   {
      polytopeToSet.getVertices().clear();

      for (int i = 0; i < convexPolytopeMessage.getVertices().size(); i++)
      {
         polytopeToSet.addVertex(new Point3D(convexPolytopeMessage.getVertices().get(i).getPoint()));
      }
   }

   public static void unpackConvexPolytope3DMessage(ConvexPolytope3DMessage convexPolytopeMessage, ConvexPolytope3D polytopeToSet)
   {
      polytopeToSet.getVertices().clear();

      for (int i = 0; i < convexPolytopeMessage.getVertices().size(); i++)
      {
         polytopeToSet.addVertex(new Point3D(convexPolytopeMessage.getVertices().get(i).getPoint()));
      }
   }

   public static void unpackCylinder3DMessage(Cylinder3DMessage cylinderMessage, Cylinder3DBasics cylinderToSet)
   {
      cylinderToSet.getPosition().set(cylinderMessage.getPosition().getPoint());
      cylinderToSet.getAxis().set(cylinderMessage.getAxis().getVector());
      cylinderToSet.setRadius(cylinderMessage.getRadius());
      cylinderToSet.setLength(cylinderMessage.getLength());
   }

   public static void unpackCapsule3DMessage(Capsule3DMessage capsuleMessage, Capsule3DBasics capsuleToSet)
   {
      capsuleToSet.getPosition().set(capsuleMessage.getPosition().getPoint());
      capsuleToSet.getAxis().set(capsuleMessage.getAxis().getVector());
      capsuleToSet.setRadius(capsuleMessage.getRadius());
      capsuleToSet.setLength(capsuleMessage.getLength());
   }

   public static void unpackEllipsoid3DMessage(Ellipsoid3DMessage ellipsoidMessage, Ellipsoid3DBasics ellipsoidToSet)
   {
      ellipsoidToSet.getPose().set(ellipsoidMessage.getPose().getPose());
      ellipsoidToSet.getRadii().set(ellipsoidMessage.getRadii().getVector());
   }

   public static void toMessage(SE3TrajectoryPointReadOnly trajectoryPoint, SE3TrajectoryPointMessage trajectoryPointMessage)
   {
      trajectoryPointMessage.setTime(trajectoryPoint.getTime());
      trajectoryPointMessage.getPosition().set(trajectoryPoint.getPosition());
      trajectoryPointMessage.getOrientation().set(trajectoryPoint.getOrientation());
      trajectoryPointMessage.getLinearVelocity().set(trajectoryPoint.getLinearVelocity());
      trajectoryPointMessage.getAngularVelocity().set(trajectoryPoint.getAngularVelocity());
   }

   public static void fromMessage(SE3TrajectoryPointMessage trajectoryPointMessage, SE3TrajectoryPoint trajectoryPoint)
   {
      trajectoryPoint.setTime(trajectoryPointMessage.getTime());
      trajectoryPoint.getPosition().set(trajectoryPointMessage.getPosition().getPoint());
      trajectoryPoint.getOrientation().set(trajectoryPointMessage.getOrientation().getQuaternion());
      trajectoryPoint.getLinearVelocity().set(trajectoryPointMessage.getLinearVelocity().getVector());
      trajectoryPoint.getAngularVelocity().set(trajectoryPointMessage.getAngularVelocity().getVector());
   }

   public static void toMessage(OneDoFTrajectoryPointReadOnly trajectoryPoint, TrajectoryPoint1DMessage trajectoryPointMessage)
   {
      trajectoryPointMessage.setTime(trajectoryPoint.getTime());
      trajectoryPointMessage.setPosition(trajectoryPoint.getPosition());
      trajectoryPointMessage.setVelocity(trajectoryPoint.getVelocity());
   }

   public static void fromMessage(TrajectoryPoint1DMessage trajectoryPointMessage, OneDoFTrajectoryPoint trajectoryPoint)
   {
      trajectoryPoint.setTime(trajectoryPointMessage.getTime());
      trajectoryPoint.setPosition(trajectoryPointMessage.getPosition());
      trajectoryPoint.setVelocity(trajectoryPointMessage.getVelocity());
   }

   public static void toMessage(Instant instant, InstantMessage instantMessage)
   {
      instantMessage.setSecondsSinceEpoch(instant.getEpochSecond());
      instantMessage.setAdditionalNanos(instant.getNano());
   }

   /**
    * Instant is immutable so there is no allocation free option.
    * If allocation free is needed, just pass around the two longs separately.
    * If needed, we could make a MutableInstant class.
    */
   public static Instant toInstant(InstantMessage instantMessage)
   {
      return Instant.ofEpochSecond(instantMessage.getSecondsSinceEpoch(), instantMessage.getAdditionalNanos());
   }

   public static void toMessage(UUID uuid, UUIDMessage uuidMessage)
   {
      uuidMessage.setLeastSignificantBits(uuid.getLeastSignificantBits());
      uuidMessage.setMostSignificantBits(uuid.getMostSignificantBits());
   }

   /**
    * UUID is immutable so there is no allocation free option.
    */
   public static UUID toUUID(UUIDMessage uuidMessage)
   {
      return new UUID(uuidMessage.getMostSignificantBits(), uuidMessage.getLeastSignificantBits());
   }

   public static void toMessage(Guid guid, GuidMessage guidMessage)
   {
      byte[] guidBytes = guid.getValue();
      System.arraycopy(guidBytes, 0, guidMessage.getPrefix(), 0, 12);
      System.arraycopy(guidBytes, 12, guidMessage.getEntity(), 0, 4);
   }

   public static void fromMessage(GuidMessage guidMessage, Guid guid)
   {
      byte[] guidBytes = new byte[16];
      System.arraycopy(guidMessage.getPrefix(), 0, guidBytes, 0, 12);
      System.arraycopy(guidMessage.getEntity(), 0, guidBytes, 12, 4);
      guid.set(guidBytes);
   }

   public static Guid toGuid(GuidMessage guidMessage)
   {
      Guid guid = new Guid();
      fromMessage(guidMessage, guid);
      return guid;
   }

   public static double calculateDelay(ImageMessage imageMessage)
   {
      return TimeTools.calculateDelay(imageMessage.getAcquisitionTime().getSecondsSinceEpoch(), imageMessage.getAcquisitionTime().getAdditionalNanos());
   }

   public static void packIDLSequence(ByteBuffer sourceBuffer, IDLByteSequence sequenceToPack)
   {
      sequenceToPack.clear();
      // A lot of data goes through here. We wish we could do a direct memcopy, but our message data is on the Java heap.
      for (int i = 0; i < sourceBuffer.limit(); i++)
      {
         sequenceToPack.add(sourceBuffer.get(i));
      }
   }

   public static void packIDLSequenceCastingIntsToBytes(ByteBuffer sourceBuffer, IDLByteSequence sequenceToPack)
   {
      sequenceToPack.clear();
      int numberOfIntegers = sourceBuffer.limit() / Integer.BYTES;
      for (int i = 0; i < numberOfIntegers; i++)
      {
         sequenceToPack.add((byte) sourceBuffer.getInt(i * Integer.BYTES));
      }
   }

   public static void packIDLSequence(ByteBuffer sourceBuffer, IDLFloatSequence sequenceToPack)
   {
      // It is important to call resetQuick, which does not set the full sequence to zeros
      sequenceToPack.clear();
      // A lot of data goes through here. We wish we could do a direct memcopy, but our message data is on the Java heap.
      int numberOfFloats = sourceBuffer.limit() / java.lang.Float.BYTES;
      for (int i = 0; i < numberOfFloats; i++)
      {
         float sourceFloat = sourceBuffer.getFloat(i * java.lang.Float.BYTES);
         sequenceToPack.add(sourceFloat);
      }
   }

   public static void extractIDLSequence(IDLByteSequence sourceIDLSequence, ByteBuffer byteBufferToPack)
   {
      byteBufferToPack.clear();
      sourceIDLSequence.copyTo(byteBufferToPack);
      byteBufferToPack.flip();
   }

   public static void extractIDLSequenceCastingBytesToInts(IDLByteSequence sourceIDLSequence, ByteBuffer byteBufferToPack)
   {
      int numberOfBytes = sourceIDLSequence.size();
      byteBufferToPack.rewind();
      byteBufferToPack.limit(byteBufferToPack.capacity());
      for (int i = 0; i < numberOfBytes; i++)
      {
         byte x = sourceIDLSequence.get(i);
         int value = Byte.toUnsignedInt(x);
         byteBufferToPack.putInt(value);
      }
      byteBufferToPack.flip();
   }

   public static void extractIDLSequence(IDLFloatSequence sourceIDLSequence, ByteBuffer byteBufferToPack)
   {
      int numberOfFloats = sourceIDLSequence.size();
      byteBufferToPack.rewind();
      byteBufferToPack.limit(byteBufferToPack.capacity());
      for (int i = 0; i < numberOfFloats; i++)
      {
         float value = sourceIDLSequence.get(i);
         byteBufferToPack.putFloat(value);
      }
      byteBufferToPack.flip();
   }

   public static PoseListMessage createPoseListMessage(Collection<Pose3DReadOnly> poses)
   {
      PoseListMessage poseListMessage = new PoseListMessage();
      packPoseListMessage(poses, poseListMessage);
      return poseListMessage;
   }

   public static <T extends Pose3DReadOnly> void packPoseListMessage(Iterable<T> poses, PoseListMessage poseListMessage)
   {
      poseListMessage.getPoses().clear();
      for (Pose3DReadOnly pose : poses)
      {
         EuclidPose3DMessage messagePose = poseListMessage.getPoses().add();
         messagePose.set(pose);
      }
   }

   public static List<Pose3D> unpackPoseListMessage(PoseListMessage poseListMessage)
   {
      ArrayList<Pose3D> poses = new ArrayList<>();
      for (int i = 0; i < poseListMessage.getPoses().size(); i++)
      {
         Pose3D pose = new Pose3D(poseListMessage.getPoses().get(i).getPose());
         poses.add(pose);
      }
      return poses;
   }

   public static Bool createBoolMessage(boolean data)
   {
      Bool bool = new Bool();
      bool.setData(data);
      return bool;
   }

   /**
    * The regular string type in our DDS messages has a maximum length of 255.
    * The get around this we declare `int8[] field_name` in the ".msg" file
    * which creates a Byte sequence 25 MB in size.
    * We use ASCII because UTF8 causes issues when publishing over DDS.
    */
   public static void packLongStringToByteSequence(String longString, IDLByteSequence byteSequence)
   {
      byteSequence.clear();
      byte[] longStringBytes = longString.getBytes(StandardCharsets.US_ASCII);
      byteSequence.addAll(longStringBytes);
   }

   public static String unpackLongStringFromByteSequence(IDLByteSequence byteSequence)
   {
      return new String(byteSequence.toByteArray(), StandardCharsets.US_ASCII);
   }

   public static int toMessage(Level level)
   {
      return level.intLevel();
   }

   public static Level fromMessage(int intLevel)
   {
      if (intLevel == Level.FATAL.intLevel())
         return Level.FATAL;
      else if (intLevel == Level.ERROR.intLevel())
         return Level.ERROR;
      else if (intLevel == Level.WARN.intLevel())
         return Level.WARN;
      else if (intLevel == Level.DEBUG.intLevel())
         return Level.DEBUG;
      else if (intLevel == Level.TRACE.intLevel())
         return Level.TRACE;
      else
         return Level.INFO;
   }

   public static int compareTime(Time a, Time b)
   {
      if (a.getSec() == b.getSec() && a.getNanosec() == b.getNanosec())
         return 0;
      else if (a.getSec() > b.getSec() || (a.getSec() == b.getSec() && a.getNanosec() > b.getNanosec()))
         return 1;

      return -1;
   }

   public static void toMessage(YoRegistry registry, YoRegistryMessage message)
   {
      IDLByteSequence data = message.getData();
      int numberOfLongs = countYoVariables(registry);
      int sizeBytes = numberOfLongs * Long.BYTES;

      data.clear();
      if (!data.ensureMinCapacity(sizeBytes))
      {
         throw new IllegalArgumentException("YoRegistry data size %d exceeds max %d".formatted(sizeBytes, data.getMaxSize()));
      }

      ByteBuffer buffer = data.getBuffer();
      toMessageInternal(registry, buffer);
   }

   private static int countYoVariables(YoRegistry registry)
   {
      int count = registry.getVariables().size();
      for (YoRegistry child : registry.getChildren())
         count += countYoVariables(child);
      return count;
   }

   private static void toMessageInternal(YoRegistry registry, ByteBuffer buffer)
   {
      for (YoVariable variable : registry.getVariables())
         buffer.putLong(variable.getValueAsLongBits());
      for (YoRegistry child : registry.getChildren())
         toMessageInternal(child, buffer);
   }

   public static void fromMessage(YoRegistryMessage message, YoRegistry registry)
   {
      ByteBuffer buffer = message.getData().getBuffer();
      buffer.limit(message.getData().size());
      buffer.position(0);
      fromMessageInternal(buffer, registry);
   }

   private static void fromMessageInternal(ByteBuffer buffer, YoRegistry registry)
   {
      for (YoVariable variable : registry.getVariables())
         variable.setValueFromLongBits(buffer.getLong());
      for (YoRegistry child : registry.getChildren())
         fromMessageInternal(buffer, child);
   }

   public static void copyData(IDLFloatSequence source, IDLFloatSequence destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
         destination.add(source.get(i));
   }
}
