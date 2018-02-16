package us.ihmc.communication.packets;

import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
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
      message.ranges = ranges;
      message.sensorId = sensorId;
      message.params = params;
      return message;
   }

   public static LidarScanMessage createLidarScanMessage(long timestamp, Point3D32 lidarPosition, Quaternion32 lidarOrientation, float[] scan)
   {
      LidarScanMessage message = new LidarScanMessage();
      message.robotTimestamp = timestamp;
      message.lidarPosition = lidarPosition;
      message.lidarOrientation = lidarOrientation;
      message.scan = scan;
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

   public static SetBooleanParameterPacket createSetBooleanParameterPacket(String parameterName, boolean parameterValue)
   {
      SetBooleanParameterPacket message = new SetBooleanParameterPacket();
      message.parameterName = parameterName;
      message.parameterValue = parameterValue;
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

   public static SetDoubleArrayParameterPacket createSetDoubleArrayParameterPacket(String parameterName, double[] parameterValue)
   {
      SetDoubleArrayParameterPacket message = new SetDoubleArrayParameterPacket();
      message.parameterName = parameterName;
      message.parameterValue = parameterValue;
      return message;
   }

   public static DetectedFacesPacket createDetectedFacesPacket(String[] ids, Point3D[] positions)
   {
      DetectedFacesPacket message = new DetectedFacesPacket();
      message.ids = ids;
      message.positions = positions;
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

   public static SetDoubleParameterPacket createSetDoubleParameterPacket(String parameterName, double parameterValue)
   {
      SetDoubleParameterPacket message = new SetDoubleParameterPacket();
      message.parameterName = parameterName;
      message.parameterValue = parameterValue;
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

   public static ParameterListPacket createParameterListPacket(List<Parameter> parameters)
   {
      ParameterListPacket message = new ParameterListPacket();
      message.parameters = parameters;
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(OneDoFJoint[] joints)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.desiredJointAngles = new float[joints.length];
      message.jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(joints);
      return message;
   }

   public static KinematicsToolboxOutputStatus createKinematicsToolboxOutputStatus(FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] newJointData,
                                                                                   boolean useQDesired)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();
      message.desiredJointAngles = new float[newJointData.length];
      message.jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(newJointData);
      message.setDesiredJointState(rootJoint, newJointData, useQDesired);
      return message;
   }

   public static SetStringParameterPacket createSetStringParameterPacket(String parameterName, String parameterValue)
   {
      SetStringParameterPacket message = new SetStringParameterPacket();
      message.parameterName = parameterName;
      message.parameterValue = parameterValue;
      return message;
   }

   public static BoundingBoxesPacket createBoundingBoxesPacket(int[] packedBoxes, String[] labels)
   {
      BoundingBoxesPacket message = new BoundingBoxesPacket();
      message.labels = labels;
      int n = packedBoxes.length / 4;
      message.boundingBoxXCoordinates = new int[n];
      message.boundingBoxYCoordinates = new int[n];
      message.boundingBoxWidths = new int[n];
      message.boundingBoxHeights = new int[n];
      for (int i = 0; i < n; i++)
      {
         message.boundingBoxXCoordinates[i] = packedBoxes[i * 4];
         message.boundingBoxYCoordinates[i] = packedBoxes[i * 4 + 1];
         message.boundingBoxWidths[i] = packedBoxes[i * 4 + 2];
         message.boundingBoxHeights[i] = packedBoxes[i * 4 + 3];
      }
      return message;
   }

   public static ControllerCrashNotificationPacket createControllerCrashNotificationPacket(ControllerCrashLocation location, String stackTrace)
   {
      ControllerCrashNotificationPacket message = new ControllerCrashNotificationPacket();
      message.location = location.toByte();
      message.stacktrace = stackTrace;
      return message;
   }

   public static StereoVisionPointCloudMessage createStereoVisionPointCloudMessage(long timestamp, float[] pointCloud, int[] colors)
   {
      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.robotTimestamp = timestamp;
      message.pointCloud = pointCloud;
      message.colors = colors;
      return message;
   }

   public static ToolboxStateMessage createToolboxStateMessage(ToolboxState requestedState)
   {
      ToolboxStateMessage message = new ToolboxStateMessage();
      message.requestedState = requestedState.toByte();
      return message;
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType)
   {
      return createRequestPlanarRegionsListMessage(requestType, null, null);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType, BoundingBox3D boundingBoxInWorldForRequest)
   {
      return createRequestPlanarRegionsListMessage(requestType, boundingBoxInWorldForRequest, null);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType, PacketDestination destination)
   {
      return createRequestPlanarRegionsListMessage(requestType, null, destination);
   }

   public static RequestPlanarRegionsListMessage createRequestPlanarRegionsListMessage(PlanarRegionsRequestType requestType, BoundingBox3D boundingBoxInWorldForRequest,
                                                                                       PacketDestination destination)
   {
      RequestPlanarRegionsListMessage message = new RequestPlanarRegionsListMessage();
      message.requestType = requestType.toByte();
      message.boundingBoxInWorldForRequest = boundingBoxInWorldForRequest;
      if (destination != null)
         message.setDestination(destination);
      return message;
   }

   public static PlanarRegionsListMessage createPlanarRegionsListMessage(List<PlanarRegionMessage> planarRegions)
   {
      PlanarRegionsListMessage message = new PlanarRegionsListMessage();
      message.planarRegions = planarRegions;
      return message;
   }

   public static <T extends Enum<T>> T fromByteToEnum(byte value, Class<T> enumType)
   {
      return enumType.getEnumConstants()[(int) value];
   }
}
