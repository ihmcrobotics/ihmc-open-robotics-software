package us.ihmc.aware.input;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Map;

import us.ihmc.aware.controller.force.QuadrupedForceControllerEvent;
import us.ihmc.aware.input.devices.XboxControllerInputDevice;
import us.ihmc.aware.packets.BodyOrientationPacket;
import us.ihmc.aware.packets.PlanarVelocityPacket;
import us.ihmc.aware.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;

public class QuadrupedTeleopNode implements InputEventCallback
{
   private final PacketCommunicator packetCommunicator;
   private final InputDevice input;
   private final Map<InputChannel, Double> channels = new EnumMap<>(InputChannel.class);

   public QuadrupedTeleopNode(NetClassList netClassList) throws IOException
   {
      // TODO: Don't use TOUCH_MODULE_PORT
      // TODO: Don't hardcode localhost
      this.packetCommunicator = PacketCommunicator
            .createTCPPacketCommunicatorClient("localhost", NetworkPorts.TOUCH_MODULE_PORT,
                  netClassList);
      this.input = new XboxControllerInputDevice();
      this.input.registerCallback(this);

      for (InputChannel axis : InputChannel.values())
      {
         channels.put(axis, 0.0);
      }
   }

   public void start() throws IOException
   {
      packetCommunicator.connect();

      // Poll indefinitely.
      input.poll();
   }

   @Override
   public void onInputEvent(InputEvent e)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(e.getChannel(), e.getValue());

      switch (e.getChannel())
      {
      case HOME_BUTTON:
         if (channels.get(InputChannel.HOME_BUTTON) == 1.0)
         {
            QuadrupedForceControllerEventPacket eventPacket = new QuadrupedForceControllerEventPacket(
                  QuadrupedForceControllerEvent.REQUEST_STAND);
            packetCommunicator.send(eventPacket);
         }
         break;
      case VIEW_BUTTON:
         break;
      case MENU_BUTTON:
         break;
      case LEFT_BUTTON:
      case RIGHT_BUTTON:
      case LEFT_TRIGGER:
      case RIGHT_TRIGGER:
      case LEFT_STICK_X:
      case LEFT_STICK_Y:
      case RIGHT_STICK_X:
      case RIGHT_STICK_Y:
         // TODO: Where should channel scaling and inversion go?
         double yaw = channels.get(InputChannel.RIGHT_STICK_X);
         double pitch = -channels.get(InputChannel.RIGHT_STICK_Y);
         double roll = channels.get(InputChannel.RIGHT_TRIGGER) - channels.get(InputChannel.LEFT_TRIGGER);
         BodyOrientationPacket orientationPacket = new BodyOrientationPacket(yaw, pitch, roll);

         double xdot = -channels.get(InputChannel.LEFT_STICK_Y);
         double ydot = -channels.get(InputChannel.LEFT_STICK_X);
         double zdot = -channels.get(InputChannel.LEFT_BUTTON) + channels.get(InputChannel.RIGHT_BUTTON);
         double adot = -channels.get(InputChannel.RIGHT_STICK_X);
         PlanarVelocityPacket velocityPacket = new PlanarVelocityPacket(xdot, ydot, adot);

         packetCommunicator.send(orientationPacket);
         packetCommunicator.send(velocityPacket);
         break;
      case BUTTON_A:
         break;
      case BUTTON_B:
         break;
      case BUTTON_X:
         break;
      case BUTTON_Y:
         break;
      case D_PAD_UP:
         break;
      case D_PAD_RIGHT:
         break;
      case D_PAD_DOWN:
         break;
      case D_PAD_LEFT:
         break;
      }
   }
}
