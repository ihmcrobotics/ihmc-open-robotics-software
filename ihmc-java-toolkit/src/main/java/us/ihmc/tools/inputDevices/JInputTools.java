package us.ihmc.tools.inputDevices;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.SystemUtils;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import us.ihmc.tools.io.printing.SystemStreamGobbler;
import us.ihmc.tools.io.printing.SystemStreamGobbler.GobbleType;

public class JInputTools
{
   private static boolean controllersHaveBeenLoaded = false;
   private static Map<ControllerType, Controller> controllerMap;
   
   public enum ControllerType
   {
      JOYSTICK_3D,
      UNKNOWN
   }
   
   private static Map<ControllerType, Controller> loadControllersByType()
   {
      Map<ControllerType, Controller> controllerMap = new HashMap<>();
      
      if (SystemUtils.IS_OS_WINDOWS_8)
      {
         System.setProperty("jinput.useDefaultPlugin", "false");
         System.setProperty("net.java.games.input.plugins", "net.java.games.input.DirectAndRawInputEnvironmentPlugin");
      }
      
      ControllerEnvironment controllerEnvironment = ControllerEnvironment.getDefaultEnvironment();
      
      SystemStreamGobbler gobbler = new SystemStreamGobbler(GobbleType.SYSTEM_OUT, GobbleType.SYSTEM_ERROR);
      Controller[] controllers = controllerEnvironment.getControllers();
      gobbler.stopGobbling();
      
      for (Controller controller : controllers)
      {
         ControllerType controllerType = getControllerType(controller);

         controllerMap.put(controllerType, controller);
      }

      return controllerMap;
   }

   public static Map<ControllerType, Controller> getControllersByType()
   {
      if (!controllersHaveBeenLoaded)
      {
         controllerMap = loadControllersByType();

         controllersHaveBeenLoaded = true;
      }

      return controllerMap;
   }
   
   public static ControllerType getControllerType(Controller controller)
   {
      boolean hasXAxis;
      boolean hasYAxis;
      boolean hasZAxis;
      boolean hasRXAxis;
      boolean hasRYAxis;
      boolean hasRZAxis;
      
      hasXAxis = controller.getComponent(Component.Identifier.Axis.X) != null;
      hasYAxis = controller.getComponent(Component.Identifier.Axis.Y) != null;
      hasZAxis = controller.getComponent(Component.Identifier.Axis.Z) != null;
      hasRXAxis = controller.getComponent(Component.Identifier.Axis.RX) != null;
      hasRYAxis = controller.getComponent(Component.Identifier.Axis.RY) != null;
      hasRZAxis = controller.getComponent(Component.Identifier.Axis.RZ) != null;
      
      boolean knownModel = controller.getName() != null && (controller.getName().equals("SpaceM") || controller.getName().equals("Space Navigator") || controller.getName().equals("SpaceMouse Pro") || controller.getName().equals("SpaceMouse Wireless Receiver"));
      
      if (hasXAxis && hasYAxis && hasZAxis && hasRXAxis && hasRYAxis && hasRZAxis && controller.getType().equals(Controller.Type.STICK) && knownModel)
         return ControllerType.JOYSTICK_3D;
      else
         return ControllerType.UNKNOWN;
   }
}
