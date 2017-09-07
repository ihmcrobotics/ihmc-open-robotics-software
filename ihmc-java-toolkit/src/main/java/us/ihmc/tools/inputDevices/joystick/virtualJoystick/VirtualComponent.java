package us.ihmc.tools.inputDevices.joystick.virtualJoystick;

import java.io.IOException;

import net.java.games.input.AbstractComponent;

public class VirtualComponent extends AbstractComponent
{
   protected VirtualComponent(String name, Identifier id)
   {
      super(name, id);
   }

   @Override
   public boolean isRelative()
   {
      return false;
   }

   @Override
   protected float poll() throws IOException
   {
      return 0;
   }
}
