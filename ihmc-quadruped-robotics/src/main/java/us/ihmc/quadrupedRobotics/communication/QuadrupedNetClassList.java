package us.ihmc.quadrupedRobotics.communication;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.communication.packets.DetectedFacesPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.quadrupedRobotics.communication.packets.BodyAngularRatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerStatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedNeckJointPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedSoleWaypointPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedXGaitSettingsPacket;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedNetClassList extends IHMCCommunicationKryoNetClassList
{
   public QuadrupedNetClassList()
   {
      super();

      registerPacketClass(QuadrupedNeckJointPositionPacket.class);
      registerPacketField(QuadrupedJointName.class);
      registerPacketField(QuadrupedJointName[].class);
      registerPacketField(double[].class);

      registerPacketClass(QuadrupedTimedStepPacket.class);
      registerPacketField(ArrayList.class);
      registerPacketField(QuadrupedTimedStep.class);


      registerPacketClass(DetectedFacesPacket.class);
      registerPacketField(String[].class);
      registerPacketField(Point3D[].class);


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

      registerPacketClass(QuadrupedForceControllerEventPacket.class);
      registerPacketField(QuadrupedForceControllerRequestedEvent.class);

      registerPacketClass(QuadrupedForceControllerStatePacket.class);
      registerPacketField(QuadrupedForceControllerEnum.class);

      registerPacketClass(QuadrupedXGaitSettingsPacket.class);
      registerPacketField(QuadrupedXGaitSettingsReadOnly.class);

      registerPacketField(String.class);
      registerPacketField(boolean.class);

      registerPacketField(String.class);
      registerPacketField(double[].class);

      registerPacketField(String.class);
      registerPacketField(double.class);

      registerPacketField(String.class);

      registerPacketClass(QuadrupedTimedStepPacket.class);
      registerPacketField(ArrayList.class);
      registerPacketField(QuadrupedTimedStep.class);
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
      
   }
}
