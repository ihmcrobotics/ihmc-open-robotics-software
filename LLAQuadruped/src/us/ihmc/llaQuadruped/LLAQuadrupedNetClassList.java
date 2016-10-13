package us.ihmc.llaQuadruped;

import java.util.ArrayList;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.quadrupedRobotics.communication.packets.BodyAngularRatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerStatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedNeckJointPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedXGaitSettingsPacket;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.quadrupedRobotics.params.BooleanParameter;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterListPacket;
import us.ihmc.quadrupedRobotics.params.RequestParameterListPacket;
import us.ihmc.quadrupedRobotics.params.SetBooleanParameterPacket;
import us.ihmc.quadrupedRobotics.params.SetDoubleArrayParameterPacket;
import us.ihmc.quadrupedRobotics.params.SetDoubleParameterPacket;
import us.ihmc.quadrupedRobotics.params.SetStringParameterPacket;
import us.ihmc.quadrupedRobotics.params.StringParameter;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LLAQuadrupedNetClassList extends IHMCCommunicationKryoNetClassList
{
   public LLAQuadrupedNetClassList()
   {
      super();
      registerPacketClass(Packet.class);

      registerPacketClass(QuadrupedNeckJointPositionPacket.class);
      registerPacketField(QuadrupedJointName.class);
      registerPacketField(QuadrupedJointName[].class);
      registerPacketField(double[].class);

      registerPacketClass(QuadrupedTimedStepPacket.class);
      registerPacketField(ArrayList.class);
      registerPacketField(QuadrupedTimedStep.class);

      registerPacketField(Point2f.class);

      registerPacketField(String[].class);
      registerPacketField(Point3d[].class);

      registerPacketClass(ComPositionPacket.class);
      registerPacketField(Point3d.class);

      registerPacketClass(ComVelocityPacket.class);
      registerPacketField(Vector3d.class);

      registerPacketClass(BodyOrientationPacket.class);
      registerPacketField(Quat4d.class);

      registerPacketClass(BodyAngularRatePacket.class);
      registerPacketField(Vector3d.class);

      registerPacketClass(PlanarVelocityPacket.class);
      registerPacketField(Vector3d.class);

      registerPacketClass(QuadrupedForceControllerEventPacket.class);
      registerPacketField(QuadrupedForceControllerRequestedEvent.class);

      registerPacketClass(QuadrupedForceControllerStatePacket.class);
      registerPacketField(QuadrupedForceControllerState.class);

      registerPacketClass(QuadrupedXGaitSettingsPacket.class);
      registerPacketField(QuadrupedXGaitSettings.class);

      registerPacketClass(SetBooleanParameterPacket.class);
      registerPacketField(String.class);
      registerPacketField(boolean.class);

      registerPacketClass(SetDoubleArrayParameterPacket.class);
      registerPacketField(String.class);
      registerPacketField(double[].class);

      registerPacketClass(SetDoubleParameterPacket.class);
      registerPacketField(String.class);
      registerPacketField(double.class);

      registerPacketClass(SetStringParameterPacket.class);
      registerPacketField(String.class);

      registerPacketClass(QuadrupedTimedStepPacket.class);
      registerPacketField(ArrayList.class);
      registerPacketField(QuadrupedTimedStep.class);
      registerPacketField(Point3d.class);
      registerPacketField(RobotQuadrant.class);
      registerPacketField(TimeInterval.class);

      registerPacketClass(RequestParameterListPacket.class);
      registerPacketClass(ParameterListPacket.class);
      registerPacketField(BooleanParameter.class);
      registerPacketField(DoubleArrayParameter .class);
      registerPacketField(DoubleParameter.class);
      registerPacketField(StringParameter.class);
   }
}
