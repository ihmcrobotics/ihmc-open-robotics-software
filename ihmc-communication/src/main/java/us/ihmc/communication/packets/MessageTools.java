package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.idl.PreallocatedList;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class MessageTools
{
   public static final boolean DEBUG = false;

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
      message.ranges.add(ranges);
      message.sensorId = sensorId;
      message.params = new LidarScanParametersMessage();
      message.params.timestamp = params.timestamp;
      message.params.sweepYawMax = params.sweepYawMax;
      message.params.sweepYawMin = params.sweepYawMin;
      message.params.heightPitchMax = params.heightPitchMax;
      message.params.heightPitchMin = params.heightPitchMin;
      message.params.timeIncrement = params.timeIncrement;
      message.params.scanTime = params.scanTime;
      message.params.minRange = params.minRange;
      message.params.maxRange = params.maxRange;
      message.params.pointsPerSweep = params.pointsPerSweep;
      message.params.scanHeight = params.scanHeight;
      return message;
   }

   public static LidarScanMessage createLidarScanMessage(long timestamp, Point3D32 lidarPosition, Quaternion32 lidarOrientation, float[] scan)
   {
      LidarScanMessage message = new LidarScanMessage();
      message.robotTimestamp = timestamp;
      message.lidarPosition = lidarPosition;
      message.lidarOrientation = lidarOrientation;
      message.scan.add(scan);
      return message;
   }

   public static ObjectDetectorResultPacket createObjectDetectorResultPacket(HeatMapPacket heatMap, BoundingBoxesPacket boundingBoxes)
   {
      ObjectDetectorResultPacket message = new ObjectDetectorResultPacket();
      message.heatMap = heatMap;
      message.boundingBoxes = boundingBoxes;
      return message;
   }

   public static UIPositionCheckerPacket createUIPositionCheckerPacket(Point3DReadOnly position)
   {
      UIPositionCheckerPacket message = new UIPositionCheckerPacket();
      message.position = new Point3D(position);
      message.orientation = null;
      return message;
   }

   public static UIPositionCheckerPacket createUIPositionCheckerPacket(Point3DReadOnly position, Quaternion orientation)
   {
      UIPositionCheckerPacket message = new UIPositionCheckerPacket();
      message.position = new Point3D(position);
      message.orientation = orientation;
      return message;
   }

   /**
    * Creates a new center of mass message.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param desiredPosition the position that center of mass should reach. The data is assumed to
    *           be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxCenterOfMassMessage createKinematicsToolboxCenterOfMassMessage(Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      message.setDesiredPosition(desiredPosition);
      return message;
   }

   public static DetectedFacesPacket createDetectedFacesPacket(String[] ids, Point3D[] positions)
   {
      DetectedFacesPacket message = new DetectedFacesPacket();
      copyData(ids, message.ids);
      copyData(positions, message.positions);
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
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for linear control only.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, Point3DReadOnly desiredPosition)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredPosition(desiredPosition);
      message.setSelectionMatrixForLinearControl();
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for angular control only.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredOrientation(desiredOrientation);
      message.setSelectionMatrixForAngularControl();
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredPose(desiredPosition, desiredOrientation);
      return message;
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public static KinematicsToolboxRigidBodyMessage createKinematicsToolboxRigidBodyMessage(RigidBody endEffector, ReferenceFrame controlFrame,
                                                                                           Point3DReadOnly desiredPosition,
                                                                                           QuaternionReadOnly desiredOrientation)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      message.setDesiredPose(desiredPosition, desiredOrientation);
      RigidBodyTransform transformToBodyFixedFrame = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(transformToBodyFixedFrame, endEffector.getBodyFixedFrame());
      message.setControlFramePose(transformToBodyFixedFrame.getTranslationVector(), transformToBodyFixedFrame.getRotationMatrix());
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
      message.set(selectionMatrix3D);
      return message;
   }

   public static WeightMatrix3DMessage createWeightMatrix3DMessage(WeightMatrix3D weightMatrix)
   {
      WeightMatrix3DMessage message = new WeightMatrix3DMessage();
      message.set(weightMatrix);
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(OneDoFJoint[] joints)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(joints);
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] newJointData,
                                                                                   boolean useQDesired)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(newJointData);
      message.setDesiredJointState(rootJoint, newJointData, useQDesired);
      return message;
   }

   public static BoundingBoxesPacket createBoundingBoxesPacket(int[] packedBoxes, String[] labels)
   {
      BoundingBoxesPacket message = new BoundingBoxesPacket();
      MessageTools.copyData(labels, message.labels);
      int n = packedBoxes.length / 4;

      for (int i = 0; i < n; i++)
      {
         message.boundingBoxXCoordinates.add(packedBoxes[i * 4]);
         message.boundingBoxYCoordinates.add(packedBoxes[i * 4 + 1]);
         message.boundingBoxWidths.add(packedBoxes[i * 4 + 2]);
         message.boundingBoxHeights.add(packedBoxes[i * 4 + 3]);
      }
      return message;
   }

   public static ControllerCrashNotificationPacket createControllerCrashNotificationPacket(ControllerCrashLocation location, String stackTrace)
   {
      ControllerCrashNotificationPacket message = new ControllerCrashNotificationPacket();
      message.controllerCrashLocation = location.toByte();
      message.stacktrace.append(stackTrace);
      return message;
   }

   public static StereoVisionPointCloudMessage createStereoVisionPointCloudMessage(long timestamp, float[] pointCloud, int[] colors)
   {
      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.robotTimestamp = timestamp;
      message.pointCloud.add(pointCloud);
      message.colors.add(colors);
      return message;
   }

   public static ToolboxStateMessage createToolboxStateMessage(ToolboxState requestedState)
   {
      ToolboxStateMessage message = new ToolboxStateMessage();
      message.requestedToolboxState = requestedState.toByte();
      return message;
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType)
   {
      return createRequestPlanarRegionsListMessage(requestType, null, null);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType,
                                                                                       BoundingBox3D boundingBoxInWorldForRequest)
   {
      return createRequestPlanarRegionsListMessage(requestType, boundingBoxInWorldForRequest, null);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType, PacketDestination destination)
   {
      return createRequestPlanarRegionsListMessage(requestType, null, destination);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType,
                                                                                       BoundingBox3D boundingBoxInWorldForRequest,
                                                                                       PacketDestination destination)
   {
      RequestPlanarRegionsListMessage message = new RequestPlanarRegionsListMessage();
      message.planarRegionsRequestType = requestType.toByte();
      message.boundingBoxInWorldForRequest = new BoundingBox3DMessage();
      if (boundingBoxInWorldForRequest != null)
      {
         message.boundingBoxInWorldForRequest.minPoint.set(boundingBoxInWorldForRequest.getMinPoint());
         message.boundingBoxInWorldForRequest.maxPoint.set(boundingBoxInWorldForRequest.getMaxPoint());
      }
      if (destination != null)
         message.setDestination(destination);
      return message;
   }

   public static PlanarRegionsListMessage createPlanarRegionsListMessage(List<PlanarRegionMessage> planarRegions)
   {
      PlanarRegionsListMessage message = new PlanarRegionsListMessage();
      copyData(planarRegions, message.planarRegions);
      return message;
   }

   public static <T extends Enum<T>> T fromByteToEnum(byte value, Class<T> enumType)
   {
      return enumType.getEnumConstants()[(int) value];
   }

   public static LidarScanParameters toLidarScanParameters(LidarScanParametersMessage message)
   {
      return new LidarScanParameters(message.pointsPerSweep, message.scanHeight, message.sweepYawMin, message.sweepYawMax, message.heightPitchMin,
                                     message.heightPitchMax, message.timeIncrement, message.minRange, message.maxRange, message.scanTime, message.timestamp);
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
    * {@link PreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    * @param <T> Should be either {@code Enum}, {@code StringBuilder}, or {@code Settable<T>}.
    * @throws IllegalArgumentException if the type {@code T} is none of the following: {@code Enum},
    *            {@code StringBuilder}, {@code Settable<T>}.
    */
   @SuppressWarnings("unchecked")
   public static <T> void copyData(PreallocatedList<T> source, PreallocatedList<T> destination)
   {
      destination.clear();

      if (source == null || source.size() == 0)
         return;

      T firstElement = destination.add();

      if (firstElement instanceof Settable)
      {
         destination.resetQuick();

         for (int i = 1; i < source.size(); i++)
         {
            ((Settable<T>) destination.add()).set(source.get(i));
         }
      }
      else if (firstElement instanceof StringBuilder)
      {
         destination.resetQuick();

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
               + PreallocatedList.class.getSimpleName() + "s declared with either of the following types: Enum, StringBuilder, and"
               + Settable.class.getSimpleName());
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link PreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the list containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static <T extends Settable<T>> void copyData(List<T> source, PreallocatedList<T> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.size(); i++)
      {
         destination.add().set(source.get(i));
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link PreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static <T extends Settable<T>> void copyData(T[] source, PreallocatedList<T> destination)
   {
      destination.clear();

      if (source == null)
         return;

      for (int i = 0; i < source.length; i++)
      {
         destination.add().set(source[i]);
      }
   }

   /**
    * Copies data from {@code source} to {@code destination} after calling
    * {@link PreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(String[] source, PreallocatedList<StringBuilder> destination)
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
    * {@link PreallocatedList#clear()} on {@code destination}.
    * 
    * @param source the array containing the data to copy. Not modified.
    * @param destination the list to copy the data into. Modified.
    */
   public static void copyData(StringBuilder[] source, PreallocatedList<StringBuilder> destination)
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

   public static <T> List<T> toList(PreallocatedList<T> original)
   {
      List<T> list = new ArrayList<>();
      for (int i = 0; i < original.size(); i++)
         list.add(original.get(i));
      return list;
   }

   public static <T extends EpsilonComparable<T>> boolean epsilonEquals(PreallocatedList<T> listOne, PreallocatedList<T> listTwo, double epsilon)
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

   /**
    * @deprecated Should use {@code PreallocatedList.remove(int)} as soon as available.
    */
   public static <T extends Settable<T>> void removeElement(PreallocatedList<T> listToRemoveElementFrom, int indexOfElementToRemove)
   {
      if (listToRemoveElementFrom.size() == 0)
         return;
      if (indexOfElementToRemove == listToRemoveElementFrom.size() - 1)
      {
         listToRemoveElementFrom.remove();
         return;
      }

      while (indexOfElementToRemove < listToRemoveElementFrom.size() - 1)
      {
         listToRemoveElementFrom.get(indexOfElementToRemove).set(listToRemoveElementFrom.get(++indexOfElementToRemove));
      }
      listToRemoveElementFrom.remove();
   }
}
