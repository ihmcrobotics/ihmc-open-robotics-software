package us.ihmc.exampleSimulations.beetle;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

import net.java.games.input.Event;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HexapodGamePadManager implements JoystickEventListener
{
   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<XBoxOneMapping, Double>(XBoxOneMapping.class));
   
   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode;
   private final YoDouble desiredLinearVelocityX;
   private final YoDouble desiredLinearVelocityY;
   private final YoDouble desiredLinearVelocityZ;
   private final YoDouble currentOrientationPitch;
   private final YoDouble currentOrientationRoll;
   private final YoDouble desiredAngularVelocityZ;
   private final YoDouble desiredBodyHeight;
   
   public HexapodGamePadManager(Joystick device, YoRegistry registry)
   {
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, true, 0.1, 1));
      
      device.setPollInterval(10);
      
     desiredLinearVelocityX = (YoDouble) registry.findVariable("BODYdesiredLinearVelocityX");
     desiredLinearVelocityY = (YoDouble) registry.findVariable("BODYdesiredLinearVelocityY");
     desiredLinearVelocityZ = (YoDouble) registry.findVariable("BODYdesiredLinearVelocityZ");
     desiredBodyHeight = (YoDouble) registry.findVariable("BODYdesiredBodyHeight");
     
     currentOrientationPitch = (YoDouble) registry.findVariable("BODYdesiredOrientationPitch");
     currentOrientationRoll = (YoDouble) registry.findVariable("BODYdesiredOrientationRoll");
     desiredAngularVelocityZ = (YoDouble) registry.findVariable("BODYdesiredAngularVelocityZ");
     controllerCoreMode = (YoEnum<WholeBodyControllerCoreMode>) registry.findVariable("controllerCoreMode");
     
     
      
     device.addJoystickEventListener(this);
   }
   
   @Override
   public void processEvent(Event event)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      switch (XBoxOneMapping.getMapping(event))
      {
      case LEFT_BUMPER:
         break;
      case RIGHT_BUMPER:
         break;
      case LEFT_TRIGGER:
         double value = channels.get(XBoxOneMapping.LEFT_TRIGGER) - 1.0;
         currentOrientationRoll.set(value * -0.4);
         break;
      case RIGHT_TRIGGER:
         double value2 = channels.get(XBoxOneMapping.RIGHT_TRIGGER) - 1.0;
         currentOrientationRoll.set(value2 * -0.4);
         break;
      case LEFT_STICK_X:
         desiredLinearVelocityY.set(channels.get(XBoxOneMapping.LEFT_STICK_X) * 0.25);
         break;
      case LEFT_STICK_Y:
         desiredLinearVelocityX.set(channels.get(XBoxOneMapping.LEFT_STICK_Y) * 0.2);
         break;
      case RIGHT_STICK_X:
         desiredAngularVelocityZ.set(channels.get(XBoxOneMapping.RIGHT_STICK_X) * 0.5);
         break;
      case RIGHT_STICK_Y:
         currentOrientationPitch.set(channels.get(XBoxOneMapping.RIGHT_STICK_Y) * 0.2);
         break;
      case SELECT:
         controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
         break;
      case START:
         controllerCoreMode.set(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
         break;
      case DPAD_DOWN:
         desiredBodyHeight.add(-0.01);
         break;
      case DPAD_UP:
         desiredBodyHeight.add(0.01);
         break;
      case DPAD:
         if(event.getValue() == 0.25)
         {
            desiredBodyHeight.add(0.01);
         }
         else if(event.getValue() == 0.75)
         {
            desiredBodyHeight.add(-0.01);
         }
      default:
         break;
      }
   }

}
