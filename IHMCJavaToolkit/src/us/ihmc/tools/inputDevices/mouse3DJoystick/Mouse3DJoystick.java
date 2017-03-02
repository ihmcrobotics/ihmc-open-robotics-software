package us.ihmc.tools.inputDevices.mouse3DJoystick;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.inputDevices.JInputTools;
import us.ihmc.tools.inputDevices.JInputTools.ControllerType;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class Mouse3DJoystick
{
   private final long POLL_PERIOD_MILLISECONDS = 10;
   private final double POLL_PERIOD_SECONDS = POLL_PERIOD_MILLISECONDS / 1e3;
   
   public final double DEAD_ZONE = 0.001;
   
   public final Map<String, Double> controllerScales = new HashMap<>();
   
   public final double MAX_VALUE = 1.0;
   public final double MIN_VALUE = -1.0;
   
   private final double X_SIGN = 1.0;
   private final double Y_SIGN = -1.0;
   private final double Z_SIGN = -1.0;
   private final double RX_SIGN = 1.0;
   private final double RY_SIGN = 1.0;
   private final double RZ_SIGN = -1.0;
   
   private Mouse3DListenerHolder mouse3DListenerHolder = new Mouse3DListenerHolder();
   private Controller controller;
   private double scale;
   private ScheduledExecutorService scheduledExecutorService;
   private Stopwatch timer = new Stopwatch();
   
   public Mouse3DJoystick()
   {
      controllerScales.put("Space Navigator", 1.0 / 350.0);
      controllerScales.put("SpaceMouse Wireless Receiver", 1.0);
      controllerScales.put("SpaceMouse Pro", 1.0);
      controllerScales.put("SpaceM", 1.0);

      Map<ControllerType, Controller> controllersByType = JInputTools.getControllersByType();
      
      if (controllersByType.containsKey(ControllerType.JOYSTICK_3D))
      {
         controller = controllersByType.get(ControllerType.JOYSTICK_3D);
         
         PrintTools.info(this, "Using Mouse3DJoystick: " + controller.getName());
         
         scale = controllerScales.get(controller.getName());
         
         scheduledExecutorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
         scheduledExecutorService.scheduleAtFixedRate(new Mouse3DJoystickPollService(), 0, POLL_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }
   
   private class Mouse3DJoystickPollService implements Runnable
   {
      private boolean timerStarted = false;
      
      @Override
      public void run()
      {
         if (!timerStarted)
         {
            timer.start();
            timerStarted = true;
            return;
         }
         
         controller.poll();
         double lap = timer.lap();
         double timeScale = lap / POLL_PERIOD_SECONDS;

         double dx = X_SIGN * controller.getComponent(Component.Identifier.Axis.X).getPollData() * scale * timeScale;
         double dy = Y_SIGN * controller.getComponent(Component.Identifier.Axis.Y).getPollData() * scale * timeScale;
         double dz = Z_SIGN * controller.getComponent(Component.Identifier.Axis.Z).getPollData() * scale * timeScale;
         double drx = RX_SIGN * controller.getComponent(Component.Identifier.Axis.RX).getPollData() * scale * timeScale;
         double dry = RY_SIGN * controller.getComponent(Component.Identifier.Axis.RY).getPollData() * scale * timeScale;
         double drz = RZ_SIGN * controller.getComponent(Component.Identifier.Axis.RZ).getPollData() * scale * timeScale;
         
         if (Math.abs(dx) < DEAD_ZONE) dx = 0.0;
         if (Math.abs(dy) < DEAD_ZONE) dy = 0.0;
         if (Math.abs(dz) < DEAD_ZONE) dz = 0.0;
         if (Math.abs(drx) < DEAD_ZONE) drx = 0.0;
         if (Math.abs(dry) < DEAD_ZONE) dry = 0.0;
         if (Math.abs(drz) < DEAD_ZONE) drz = 0.0;
         
         if (!(dx == 0.0 && dy == 0.0 && dz == 0.0 && drx == 0.0 && dry == 0.0 && drz == 0.0))
            mouse3DListenerHolder.mouseDragged(dx, dy, dz, drx, dry, drz);
      }
   }
   
   public void addMouse3DListener(Mouse3DListener mouse3DListener)
   {
      mouse3DListenerHolder.addMouse3DListener(mouse3DListener);
   }
   
   public void stopPolling()
   {
      if (scheduledExecutorService != null)
         scheduledExecutorService.shutdown();
   }
}
