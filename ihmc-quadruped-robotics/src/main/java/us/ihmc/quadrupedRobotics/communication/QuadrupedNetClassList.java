package us.ihmc.quadrupedRobotics.communication;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.*;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.RecyclingArrayListPubSub;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.quadrupedRobotics.communication.packets.*;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedNetClassList extends NetClassList
{
   public QuadrupedNetClassList()
   {
      registerPacketField(TDoubleArrayList.class);
      registerPacketField(TIntArrayList.class);
      registerPacketField(TLongArrayList.class);
      registerPacketField(TFloatArrayList.class);
      registerPacketField(TByteArrayList.class);
      registerPacketField(IDLSequence.Object.class);
      registerPacketField(IDLSequence.Float.class);
      registerPacketField(IDLSequence.Boolean.class);
      registerPacketField(IDLSequence.Double.class);
      registerPacketField(IDLSequence.Integer.class);
      registerPacketField(IDLSequence.Byte.class);
      registerPacketField(IDLSequence.Long.class);
      registerPacketField(IDLSequence.StringBuilderHolder.class);
      registerPacketField(TopicDataType.class);
      registerPacketField(RecyclingArrayListPubSub.class);
      registerPacketField(us.ihmc.idl.CDR.class);
      registerPacketField(ArrayList.class);


      registerPacketField(QuadrupedTimedStepListMessagePubSubType.class);



      registerPacketClass(Packet.class);

      registerPacketClass(MessageCollection.class);
      registerPacketClass(MessageCollectionNotification.class);

      registerPacketField(StringBuilder.class);
      registerPacketField(StringBuilder[].class);

      // Video data
      registerPacketClass(VideoPacket.class);
      registerPacketField(VideoPacket.class);
      registerPacketClass(SimulatedLidarScanPacket.class);
      registerPacketField(IntrinsicParametersMessage.class);
      registerPacketField(LidarScanParametersMessage.class);

      registerPacketClass(IMUPacket.class);
      registerPacketField(IMUPacket[].class);
      registerPacketField(IMUPacket.class);

      registerPacketField(byte[].class);
      registerPacketField(Point3D.class);
      registerPacketField(Quaternion.class);
      registerPacketField(TimeStampedTransform3D.class);

      registerPacketField(PacketDestination.class);


      // Joint data
      registerPacketClass(RobotConfigurationData.class);
      registerPacketClass(QuadrupedNeckJointPositionPacket.class);
      registerPacketField(QuadrupedJointName.class);
      registerPacketField(QuadrupedJointName[].class);
      registerPacketField(double[].class);

      // Footstep data
      registerPacketField(QuadrupedTimedStepListMessage.class);
      registerPacketClass(QuadrupedTimedStepListMessage.class);

      registerPacketField(ArrayList.class);
      registerPacketField(Point3D.class);
      registerPacketField(RobotQuadrant.class);
      registerPacketField(TimeInterval.class);

      registerPacketClass(QuadrupedSoleWaypointPacket.class);
      registerPacketField(QuadrupedSoleWaypointList.class);
      registerPacketField(QuadrantDependentList.class);
      registerPacketField(RobotQuadrant[][].class);
      registerPacketField(RobotQuadrant[].class);
      registerPacketField(ArrayList.class);
      registerPacketField(SoleWaypoint.class);
      registerPacketField(Point3D.class);
      registerPacketField(Vector3D.class);
      registerPacketField(Double.class);
      registerPacketField(Object[].class);


      // Head Data
      registerPacketClass(DetectedFacesPacket.class);
      registerPacketField(String[].class);
      registerPacketField(Point3D[].class);


      // Body Orientation
      registerPacketClass(ComPositionPacket.class);
      registerPacketField(Point3D.class);

      registerPacketClass(ComVelocityPacket.class);
      registerPacketField(Vector3D.class);

      registerPacketClass(BodyOrientationPacket.class);
      registerPacketField(Quaternion.class);

      registerPacketClass(BodyAngularRatePacket.class);
      registerPacketField(Vector3D.class);

      registerPacketClass(PlanarVelocityPacket.class);
      registerPacketField(Vector3D.class);

      // Events and controller statesregisterPacketClass(QuadrupedXGaitSettingsPacket.class); // fixme make a message
      registerPacketField(QuadrupedXGaitSettingsReadOnly.class); // fixme make a message

      registerPacketClass(QuadrupedRequestedControllerStateMessage.class);
      registerPacketClass(QuadrupedRequestedSteppingStateMessage.class);

      registerPacketClass(QuadrupedSteppingStateChangeMessage.class);
      registerPacketClass(QuadrupedControllerStateChangeMessage.class);

      registerPacketField(String.class);
      registerPacketField(boolean.class);

      registerPacketField(String.class);
      registerPacketField(double[].class);

      registerPacketField(String.class);
      registerPacketField(double.class);

      registerPacketField(String.class);
   }
}
