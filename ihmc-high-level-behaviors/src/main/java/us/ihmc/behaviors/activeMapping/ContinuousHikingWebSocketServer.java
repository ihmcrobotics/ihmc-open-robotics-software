package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import com.google.gson.Gson;
import org.java_websocket.server.WebSocketServer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicReference;

class ContinuousHikingCommand
{
   public boolean enable_continuous_hiking;
   public long steps_before_safety_stop;
   public boolean walk_forwards;
   public boolean side_step;
   public boolean left_direction;
   public boolean square_up_to_goal;
   public boolean use_astar_footstep_planner;
   public boolean use_monte_carlo_footstep_planner;
   public boolean use_previous_plan_as_reference;
   public boolean use_monte_carlo_plan_as_reference;
   public boolean use_joystick_controller;
   public double forward_value;
   public boolean walk_backwards;
   public double lateral_value;
   public double turning_value;
}

public class ContinuousHikingWebSocketServer extends WebSocketServer
{
   private final Gson gson = new Gson();
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;

   public ContinuousHikingWebSocketServer(int port, AtomicReference<ContinuousHikingCommandMessage> commandMessage)
   {
      super(new InetSocketAddress(port));
      this.commandMessage = commandMessage;
   }

   @Override
   public void onOpen(WebSocket conn, ClientHandshake handshake)
   {
      System.out.println("Client connected: " + conn.getRemoteSocketAddress());
   }

   @Override
   public void onClose(WebSocket conn, int code, String reason, boolean remote)
   {
      System.out.println("Client disconnected: " + conn.getRemoteSocketAddress());
   }

   @Override
   public void onMessage(WebSocket conn, String message)
   {
      if ("STOP_SERVER".equalsIgnoreCase(message.trim()))
      {
         conn.send("Stopping server...");
         new Thread(() -> {
            try
            {
               this.stop();
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }).start();
         return;
      }

      try
      {
         ContinuousHikingCommand parsed = gson.fromJson(message, ContinuousHikingCommand.class);
         commandMessage.get().setEnableContinuousHiking(parsed.enable_continuous_hiking);
         commandMessage.get().setStepsBeforeSafetyStop(parsed.steps_before_safety_stop);
         commandMessage.get().setWalkForwards(parsed.walk_forwards);
         commandMessage.get().setSideStep(parsed.side_step);
         commandMessage.get().setLeftDirection(parsed.left_direction);
         commandMessage.get().setTurningValue(parsed.turning_value);
         commandMessage.get().setLateralValue(parsed.lateral_value);
         commandMessage.get().setSquareUpToGoal(parsed.square_up_to_goal);
         commandMessage.get().setUseAstarFootstepPlanner(parsed.use_astar_footstep_planner);
         commandMessage.get().setUseMonteCarloFootstepPlanner(parsed.use_monte_carlo_footstep_planner);
         commandMessage.get().setUsePreviousPlanAsReference(parsed.use_previous_plan_as_reference);
         commandMessage.get().setUseJoystickController(parsed.use_joystick_controller);
         commandMessage.get().setForwardValue(parsed.forward_value);
         commandMessage.get().setWalkBackwards(parsed.walk_backwards);
         conn.send("ACK: Command updated");
      }
      catch (Exception e)
      {
         conn.send("ERROR: Failed to parse command");
         e.printStackTrace();
      }
   }

   @Override
   public void onError(WebSocket conn, Exception ex)
   {
      ex.printStackTrace();
   }

   @Override
   public void onStart()
   {

   }

   public static void main(String[] args)
   {
      int port = 8765;
      AtomicReference<ContinuousHikingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousHikingCommandMessage());
      ContinuousHikingWebSocketServer server = new ContinuousHikingWebSocketServer(port, commandMessage);
      server.start();
      System.out.println("WebSocket server started on port " + port);
   }
}
