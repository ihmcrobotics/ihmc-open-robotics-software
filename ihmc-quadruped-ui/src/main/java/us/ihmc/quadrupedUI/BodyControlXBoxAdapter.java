package us.ihmc.quadrupedUI;

import net.java.games.input.Event;
import us.ihmc.quadrupedPlanning.input.InputValueIntegrator;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

public class BodyControlXBoxAdapter implements JoystickEventListener
{
   private static final double bodyHeightVelocity = 0.03;
   private static final double maxBodyYaw = 0.2;
   private static final double maxBodyPitch = 0.15;
   private static final double maxBodyRoll = 0.15;
   private static final double maxBodyTranslation = 0.1;

   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<>(XBoxOneMapping.class));
   private InputValueIntegrator bodyHeight;
   private double commandedBodyYaw = 0.0;
   private double commandedBodyPitch = 0.0;
   private double commandedBodyRoll = 0.0;
   private double commandedBodyTranslationX = 0.0;
   private double commandedBodyTranslationY = 0.0;

   public BodyControlXBoxAdapter(double nominalBodyHeight) throws JoystickNotFoundException
   {
      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      this.bodyHeight = new InputValueIntegrator(BodyPoseController.DT, nominalBodyHeight);

      for (XBoxOneMapping channel : XBoxOneMapping.values)
         channels.put(channel, 0.0);

      joystick.addJoystickEventListener(this);
      joystick.setPollInterval(10);

      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.1, 1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.1, 1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, false, 0.1, 1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, false, 0.1, 1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, false, 0.1, 1));
      joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, false, 0.1, 1));
   }

   @Override
   public void processEvent(Event event)
   {
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());
   }

   void update()
   {
      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         bodyHeight.update(bodyHeightVelocity);
      }
      else if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         bodyHeight.update(-bodyHeightVelocity);
      }

      double bodyYawLeft = channels.get(XBoxOneMapping.LEFT_TRIGGER) * maxBodyYaw;
      double bodyYawRight = channels.get(XBoxOneMapping.RIGHT_TRIGGER) * maxBodyYaw;
      commandedBodyYaw = bodyYawLeft - bodyYawRight;

      commandedBodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * maxBodyPitch;
      commandedBodyRoll = channels.get(XBoxOneMapping.RIGHT_STICK_X) * maxBodyRoll;

      commandedBodyTranslationX = channels.get(XBoxOneMapping.LEFT_STICK_Y) * maxBodyTranslation;
      commandedBodyTranslationY = channels.get(XBoxOneMapping.LEFT_STICK_X) * maxBodyTranslation;
   }

   double getCommandedBodyHeight()
   {
      return bodyHeight.value();
   }

   double getCommandedBodyYaw()
   {
      return commandedBodyYaw;
   }

   double getCommandedBodyPitch()
   {
      return commandedBodyPitch;
   }

   double getCommandedBodyRoll()
   {
      return commandedBodyRoll;
   }

   double getCommandedBodyTranslationX()
   {
      return commandedBodyTranslationX;
   }

   double getCommandedBodyTranslationY()
   {
      return commandedBodyTranslationY;
   }
}
