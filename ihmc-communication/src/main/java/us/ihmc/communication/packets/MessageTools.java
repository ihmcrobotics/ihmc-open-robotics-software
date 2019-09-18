package us.ihmc.communication.packets;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import controller_msgs.msg.dds.BoundingBoxesPacket;
import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.DetectedFacesPacket;
import controller_msgs.msg.dds.HeatMapPacket;
import controller_msgs.msg.dds.InvalidPacketNotificationPacket;
import controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.LidarScanParametersMessage;
import controller_msgs.msg.dds.ObjectDetectorResultPacket;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.SimulatedLidarScanPacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import controller_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import controller_msgs.msg.dds.WeightMatrix3DMessage;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

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

   public static SimulatedLidarScanPacket createSimulatedLidarScanPacket(int sensorId, LidarScanParameters params, float[] ranges)
   {
      SimulatedLidarScanPacket message = new SimulatedLidarScanPacket();
      message.getRanges().add(ranges);
      message.setSensorId(sensorId);
      message.getLidarScanParameters().setTimestamp(params.timestamp);
      message.getLidarScanParameters().setSweepYawMax(params.sweepYawMax);
      message.getLidarScanParameters().setSweepYawMin(params.sweepYawMin);
      message.getLidarScanParameters().setHeightPitchMax(params.heightPitchMax);
      message.getLidarScanParameters().setHeightPitchMin(params.heightPitchMin);
      message.getLidarScanParameters().setTimeIncrement(params.timeIncrement);
      message.getLidarScanParameters().setScanTime(params.scanTime);
      message.getLidarScanParameters().setMinRange(params.minRange);
      message.getLidarScanParameters().setMaxRange(params.maxRange);
      message.getLidarScanParameters().setPointsPerSweep(params.pointsPerSweep);
      message.getLidarScanParameters().setScanHeight(params.scanHeight);
      return message;
   }

   public static InvalidPacketNotificationPacket createInvalidPacketNotificationPacket(Class<?> packetClass, String errorMessage)
   {
      InvalidPacketNotificationPacket message = new InvalidPacketNotificationPacket();
      message.setPacketClassSimpleName(packetClass.getSimpleName());
      message.setErrorMessage(errorMessage);
      return message;
   }

   public static LidarScanMessage createLidarScanMessage(long timestamp, Point3D32 lidarPosition, Quaternion32 lidarOrientation, float[] scan)
   {
      LidarScanMessage message = new LidarScanMessage();
      message.setRobotTimestamp(timestamp);
      message.getLidarPosition().set(lidarPosition);
      message.getLidarOrientation().set(lidarOrientation);
      message.getScan().add(scan);
      return message;
   }

   public static ObjectDetectorResultPacket createObjectDetectorResultPacket(HeatMapPacket heatMap, BoundingBoxesPacket boundingBoxes)
   {
      ObjectDetectorResultPacket message = new ObjectDetectorResultPacket();
      message.getHeatMap().set(heatMap);
      message.getBoundingBoxes().set(boundingBoxes);
      return message;
   }

   public static UIPositionCheckerPacket createUIPositionCheckerPacket(Point3DReadOnly position)
   {
      UIPositionCheckerPacket message = new UIPositionCheckerPacket();
      message.getPosition().set(position);
      message.getOrientation().setToNaN();
      return message;
   }

   public static UIPositionCheckerPacket createUIPositionCheckerPacket(Point3DReadOnly position, Quaternion orientation)
   {
      UIPositionCheckerPacket message = new UIPositionCheckerPacket();
      message.getPosition().set(position);
      message.getOrientation().set(orientation);
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
    *           expressed in world frame. Not modified.
    */
   public static KinematicsToolboxCenterOfMassMessage createKinematicsToolboxCenterOfMassMessage(Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      message.getDesiredPositionInWorld().set(desiredPosition);
      return message;
   }

   public static DetectedFacesPacket createDetectedFacesPacket(String[] ids, Point3D[] positions)
   {
      DetectedFacesPacket message = new DetectedFacesPacket();
      copyData(ids, message.getIds());
      copyData(positions, message.getPositions());
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
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyBasics endEffector)
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
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyBasics endEffector, Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredPositionInWorld().set(desiredPosition);
      message.getAngularSelectionMatrix().setXSelected(false);
      message.getAngularSelectionMatrix().setYSelected(false);
      message.getAngularSelectionMatrix().setZSelected(false);
      message.getLinearSelectionMatrix().setXSelected(true);
      message.getLinearSelectionMatrix().setYSelected(true);
      message.getLinearSelectionMatrix().setZSelected(true);
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
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyBasics endEffector, QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredOrientationInWorld().set(desiredOrientation);
      message.getAngularSelectionMatrix().setXSelected(true);
      message.getAngularSelectionMatrix().setYSelected(true);
      message.getAngularSelectionMatrix().setZSelected(true);
      message.getLinearSelectionMatrix().setXSelected(false);
      message.getLinearSelectionMatrix().setYSelected(false);
      message.getLinearSelectionMatrix().setZSelected(false);
      return message;
   }

   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyBasics endEffector, Pose3DReadOnly desiredPose)
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
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyBasics endEffector, Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
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
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBodyBasics endEffector, ReferenceFrame controlFrame,
                                                                                           Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(endEffector.hashCode());
      message.getDesiredPositionInWorld().set(desiredPosition);
      message.getDesiredOrientationInWorld().set(desiredOrientation);
      RigidBodyTransform transformToBodyFixedFrame = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(transformToBodyFixedFrame, endEffector.getBodyFixedFrame());
      message.getControlFramePositionInEndEffector().set(transformToBodyFixedFrame.getTranslationVector());
      message.getControlFrameOrientationInEndEffector().set(transformToBodyFixedFrame.getRotationMatrix());
      return message;
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
      message.setSelectionFrameId(MessageTools.toFrameId(selectionMatrix3D.getSelectionFrame()));
      message.setXSelected(selectionMatrix3D.isXSelected());
      message.setYSelected(selectionMatrix3D.isYSelected());
      message.setZSelected(selectionMatrix3D.isZSelected());
      return message;
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
      message.setWeightFrameId(MessageTools.toFrameId(null));
      message.setXWeight(weight);
      message.setYWeight(weight);
      message.setZWeight(weight);
      return message;
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
               message.getRobotConfigurations().add()
                      .set(interpolateMessages(previewFrames.get(firstInputFrameIndex), previewFrames.get(secondInputFrameIndex), alpha));
            }
         }
      }
      return message;
   }

   public static BoundingBoxesPacket createBoundingBoxesPacket(int[] packedBoxes, String[] labels)
   {
      BoundingBoxesPacket message = new BoundingBoxesPacket();
      MessageTools.copyData(labels, message.getLabels());
      int n = packedBoxes.length / 4;

      for (int i = 0; i < n; i++)
      {
         message.getBoundingBoxesXCoordinates().add(packedBoxes[i * 4]);
         message.getBoundingBoxesYCoordinates().add(packedBoxes[i * 4 + 1]);
         message.getBoundingBoxesWidths().add(packedBoxes[i * 4 + 2]);
         message.getBoundingBoxesHeights().add(packedBoxes[i * 4 + 3]);
      }
      return message;
   }

   public static ControllerCrashNotificationPacket createControllerCrashNotificationPacket(ControllerCrashLocation location, String stackTrace)
   {
      ControllerCrashNotificationPacket message = new ControllerCrashNotificationPacket();
      message.setControllerCrashLocation(location.toByte());
      message.setStacktrace(stackTrace);
      return message;
   }

   public static StereoVisionPointCloudMessage createStereoVisionPointCloudMessage(long timestamp, float[] pointCloud, int[] colors)
   {
      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.setTimestamp(timestamp);
      message.getPointCloud().add(pointCloud);
      message.getColors().add(colors);
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

   public static LidarScanParameters toLidarScanParameters(LidarScanParametersMessage message)
   {
      return new LidarScanParameters(message.getPointsPerSweep(), message.getScanHeight(), message.getSweepYawMin(), message.getSweepYawMax(),
                                     message.getHeightPitchMin(), message.getHeightPitchMax(), message.getTimeIncrement(), message.getMinRange(),
                                     message.getMaxRange(), message.getScanTime(), message.getTimestamp());
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link TByteArrayList#reset()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
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
    * @param source the list containing the data to copy. Not modified.
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
    * @param source the list containing the data to copy. Not modified.
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
    * @param source the list containing the data to copy. Not modified.
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
    * @param source the list containing the data to copy. Not modified.
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

   /**
    * Performs a deep copy of the data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    * @param <T> Should be either {@code Enum}, {@code StringBuilder}, or {@code Settable<T>}.
    * @throws IllegalArgumentException if the type {@code T} is none of the following: {@code Enum},
    *            {@code StringBuilder}, {@code Settable<T>}.
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
         throw new IllegalArgumentException(MessageTools.class.getSimpleName() + ".copyData(...) can only be used with "
               + RecyclingArrayList.class.getSimpleName() + "s declared with either of the following types: Enum, StringBuilder, and"
               + Settable.class.getSimpleName());
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static <T extends Settable<T>> void copyData(T[] source, RecyclingArrayList<T> destination)
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
         PrintTools.error("Caught exception while copying data from array of length: " + source.length);
         throw e;
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link RecyclingArrayList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
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
    * @param source the array containing the data to copy. Not modified.
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
         return NameBasedHashCodeTools.NULL_HASHCODE;
      else
         return referenceFrame.hashCode();
   }

   public static Point3D32[] unpackPointCloud32(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      Point3D32[] points = new Point3D32[stereoVisionPointCloudMessage.getPointCloud().size() / 3];
      for (int index = 0; index < stereoVisionPointCloudMessage.getPointCloud().size() / 3; index++)
      {
         Point3D32 scanPoint = new Point3D32();
         scanPoint.setX(stereoVisionPointCloudMessage.getPointCloud().get(3 * index + 0));
         scanPoint.setY(stereoVisionPointCloudMessage.getPointCloud().get(3 * index + 1));
         scanPoint.setZ(stereoVisionPointCloudMessage.getPointCloud().get(3 * index + 2));
         points[index] = scanPoint;
      }
      return points;
   }

   public static Color[] unpackPointCloudColors(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      Color[] colors = new Color[stereoVisionPointCloudMessage.getColors().size()];

      for (int i = 0; i < stereoVisionPointCloudMessage.getColors().size(); i++)
      {
         colors[i] = new Color(stereoVisionPointCloudMessage.getColors().get(i));
      }

      return colors;
   }

   public static void unpackDesiredJointState(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus, FloatingJointBasics rootJointToUpdate,
                                              OneDoFJointBasics[] jointsToUpdate)
   {
      int jointNameHash = Arrays.hashCode(jointsToUpdate);

      if (jointNameHash != kinematicsToolboxOutputStatus.getJointNameHash())
         throw new RuntimeException("The robots are different.");

      for (int i = 0; i < kinematicsToolboxOutputStatus.getDesiredJointAngles().size(); i++)
         jointsToUpdate[i].setQ(kinematicsToolboxOutputStatus.getDesiredJointAngles().get(i));

      rootJointToUpdate.setJointPosition(kinematicsToolboxOutputStatus.getDesiredRootTranslation());
      rootJointToUpdate.setJointOrientation(kinematicsToolboxOutputStatus.getDesiredRootOrientation());
   }

   public static void packDesiredJointState(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus, FloatingJointBasics rootJoint,
                                            OneDoFJointBasics[] newJointData)
   {
      int jointNameHash = Arrays.hashCode(newJointData);

      if (jointNameHash != kinematicsToolboxOutputStatus.getJointNameHash())
         throw new RuntimeException("The robots are different.");

      kinematicsToolboxOutputStatus.getDesiredJointAngles().reset();

      for (int i = 0; i < newJointData.length; i++)
      {
         kinematicsToolboxOutputStatus.getDesiredJointAngles().add((float) newJointData[i].getQ());
      }

      if (rootJoint != null)
      {
         kinematicsToolboxOutputStatus.getDesiredRootTranslation().set(rootJoint.getJointPose().getPosition());
         kinematicsToolboxOutputStatus.getDesiredRootOrientation().set(rootJoint.getJointPose().getOrientation());
      }
   }

   public static KinematicsToolboxOutputStatus interpolateMessages(KinematicsToolboxOutputStatus outputStatusOne, KinematicsToolboxOutputStatus outputStatusTwo,
                                                                   double alpha)
   {
      if (outputStatusOne.getJointNameHash() != outputStatusTwo.getJointNameHash())
         throw new RuntimeException("Output status are not compatible.");

      KinematicsToolboxOutputStatus interplateOutputStatus = new KinematicsToolboxOutputStatus();

      TFloatArrayList jointAngles1 = outputStatusOne.getDesiredJointAngles();
      TFloatArrayList jointAngles2 = outputStatusTwo.getDesiredJointAngles();

      for (int i = 0; i < jointAngles1.size(); i++)
      {
         interplateOutputStatus.getDesiredJointAngles().add((float) EuclidCoreTools.interpolate(jointAngles1.get(i), jointAngles2.get(i), alpha));
      }

      Vector3D rootTranslation1 = outputStatusOne.getDesiredRootTranslation();
      Vector3D rootTranslation2 = outputStatusTwo.getDesiredRootTranslation();
      Quaternion rootOrientation1 = outputStatusOne.getDesiredRootOrientation();
      Quaternion rootOrientation2 = outputStatusTwo.getDesiredRootOrientation();

      interplateOutputStatus.getDesiredRootTranslation().interpolate(rootTranslation1, rootTranslation2, alpha);
      interplateOutputStatus.getDesiredRootOrientation().interpolate(rootOrientation1, rootOrientation2, alpha);

      interplateOutputStatus.setJointNameHash(outputStatusOne.getJointNameHash());

      return interplateOutputStatus;
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
    * @param rootJointPosition the privileged root joint position. Not modified.
    * @param rootJointOrientation the privileged root joint orientation. Not modified.
    * @param jointHashCodes allows to safely identify to which joint each angle in
    *           {@link #privilegedJointAngles} belongs to. The hash code can be obtained from
    *           {@link OneDoFJointBasics#hashCode()}. Not modified.
    * @param jointAngles the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and {@code jointHashCodes}
    *            are different.
    */
   public static void packPrivilegedRobotConfiguration(KinematicsToolboxConfigurationMessage kinematicsToolboxConfigurationMessage,
                                                       Tuple3DReadOnly rootJointPosition, QuaternionReadOnly rootJointOrientation, int[] jointHashCodes,
                                                       float[] jointAngles)
   {
      kinematicsToolboxConfigurationMessage.getPrivilegedRootJointPosition().set(rootJointPosition);
      kinematicsToolboxConfigurationMessage.getPrivilegedRootJointOrientation().set(rootJointOrientation);
      MessageTools.packPrivilegedJointAngles(kinematicsToolboxConfigurationMessage, jointHashCodes, jointAngles);
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
    *           {@link #privilegedJointAngles} belongs to. The hash code can be obtained from
    *           {@link OneDoFJointBasics#hashCode()}. Not modified.
    * @param jointAngles the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and {@code jointHashCodes}
    *            are different.
    */
   public static void packPrivilegedJointAngles(KinematicsToolboxConfigurationMessage kinematicsToolboxConfigurationMessage, int[] jointHashCodes,
                                                float[] jointAngles)
   {
      if (jointHashCodes.length != jointAngles.length)
         throw new IllegalArgumentException("The two arrays jointAngles and jointHashCodes have to be of same length.");

      kinematicsToolboxConfigurationMessage.getPrivilegedJointHashCodes().reset();
      kinematicsToolboxConfigurationMessage.getPrivilegedJointHashCodes().add(jointHashCodes);
      kinematicsToolboxConfigurationMessage.getPrivilegedJointAngles().reset();
      kinematicsToolboxConfigurationMessage.getPrivilegedJointAngles().add(jointAngles);
   }

   public static void packScan(LidarScanMessage lidarScanMessage, Point3DReadOnly[] scan)
   {
      lidarScanMessage.getScan().reset();

      for (Point3DReadOnly scanPoint : scan)
      {
         lidarScanMessage.getScan().add((float) scanPoint.getX());
         lidarScanMessage.getScan().add((float) scanPoint.getY());
         lidarScanMessage.getScan().add((float) scanPoint.getZ());
      }
   }

   public static void unpackScanPoint(LidarScanMessage lidarScanMessage, int index, Point3DBasics scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(lidarScanMessage.getScan().get(index++));
      scanPointToPack.setY(lidarScanMessage.getScan().get(index++));
      scanPointToPack.setZ(lidarScanMessage.getScan().get(index++));
   }

   public static Point3D[] unpackScanPoint3ds(LidarScanMessage lidarScanMessage)
   {
      int numberOfScanPoints = lidarScanMessage.getScan().size() / 3;
      Point3D[] scanPoints = new Point3D[numberOfScanPoints];
      for (int index = 0; index < numberOfScanPoints; index++)
      {
         Point3D scanPoint1 = new Point3D();
         MessageTools.unpackScanPoint(lidarScanMessage, index, scanPoint1);
         Point3D scanPoint = scanPoint1;
         scanPoints[index] = scanPoint;
      }
      return scanPoints;
   }

   public static void unpackScanPoint(StereoVisionPointCloudMessage stereoVisionPointCloudMessage, int index, Point3DBasics scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(stereoVisionPointCloudMessage.getPointCloud().get(index++));
      scanPointToPack.setY(stereoVisionPointCloudMessage.getPointCloud().get(index++));
      scanPointToPack.setZ(stereoVisionPointCloudMessage.getPointCloud().get(index++));
   }
   
   public static Point3D[] unpackScanPoint3ds(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      int numberOfScanPoints = stereoVisionPointCloudMessage.point_cloud_.size() / 3;
      Point3D[] scanPoints = new Point3D[numberOfScanPoints];
      for (int index = 0; index < numberOfScanPoints; index++)
      {
         Point3D scanPoint1 = new Point3D();
         MessageTools.unpackScanPoint(stereoVisionPointCloudMessage, index, scanPoint1);
         Point3D scanPoint = scanPoint1;
         scanPoints[index] = scanPoint;
      }
      return scanPoints;
   }
}
