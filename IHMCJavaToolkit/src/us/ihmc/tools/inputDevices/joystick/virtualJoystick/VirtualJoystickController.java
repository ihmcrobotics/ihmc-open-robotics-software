package us.ihmc.tools.inputDevices.joystick.virtualJoystick;

import java.io.IOException;
import java.util.HashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

import net.java.games.input.AbstractController;
import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.Event;
import net.java.games.input.Rumbler;

public class VirtualJoystickController extends AbstractController
{
   private static final Identifier[] identifiers = new Identifier[10];
   private static final Component[] components = new Component[identifiers.length];
   private static final HashMap<Identifier, Component> identifierToComponentMap = new HashMap<>();
   static
   {
      identifiers[0] = Identifier.Axis.X;
      identifiers[1] = Identifier.Axis.Y;
      identifiers[2] = Identifier.Axis.Z;
      identifiers[3] = Identifier.Axis.RX;
      identifiers[4] = Identifier.Axis.RY;
      identifiers[5] = Identifier.Axis.RZ;
      identifiers[6] = Identifier.Axis.SLIDER;
      identifiers[7] = Identifier.Axis.POV;
      identifiers[8] = Identifier.Button._0;
      identifiers[9] = Identifier.Button._1;
      
      for (int i = 0; i < identifiers.length; i++)
      {
         components[i] = new VirtualComponent(identifiers[i].getName(), identifiers[i]);
         identifierToComponentMap.put(identifiers[i], components[i]);
      }
   }
   
   private final ConcurrentLinkedQueue<Event> eventQueue = new ConcurrentLinkedQueue<>();
   
   protected VirtualJoystickController(String name)
   {
      super(name, components, new Controller[0], new Rumbler[0]);
   }
   
   public void queueEvent(Identifier identifier, double value)
   {
      Event event = new Event();
      event.set(identifierToComponentMap.get(identifier), (float) value, System.nanoTime());
      eventQueue.add(event);
   }

   @Override
   protected boolean getNextDeviceEvent(Event event) throws IOException
   {
      if (!eventQueue.isEmpty())
      {
         event.set(eventQueue.poll());
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public Type getType()
   {
      return Controller.Type.STICK;
   }
}
