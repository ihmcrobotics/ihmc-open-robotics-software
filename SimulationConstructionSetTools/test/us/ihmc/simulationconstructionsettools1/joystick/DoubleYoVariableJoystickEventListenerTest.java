package us.ihmc.simulationconstructionsettools1.joystick;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class DoubleYoVariableJoystickEventListenerTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMinMaxAverage()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DoubleYoVariable variable = new DoubleYoVariable("test", registry);
      TestComponent component = new TestComponent();
      component.setAnalog(true);
      component.setDeadZone(0.0f);

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         Random random = new Random(12342L);
         double min = random.nextDouble() - 0.5;
         double max = min + random.nextDouble();
         double deadZone = 0.0;
         boolean signFlip = false;
         DoubleYoVariableJoystickEventListener listener = new DoubleYoVariableJoystickEventListener(variable, component, min, max, deadZone, signFlip);

         Event event = new Event();

         event.set(component, -1.0f, 0);
         listener.processEvent(event);
         assertEquals(min, variable.getDoubleValue(), 0.0);

         event.set(component, 1.0f, 0);
         listener.processEvent(event);
         assertEquals(max, variable.getDoubleValue(), 0.0);
         
         event.set(component, 0.0f, 0);
         listener.processEvent(event);
         assertEquals((min + max) / 2.0, variable.getDoubleValue(), 0.0);         
      }
   }

   private final class TestComponent implements Component
   {
      private boolean isRelative;
      private boolean isAnalog;
      private float pollData;
      private String name;
      private Identifier identifier;
      private float deadZone;

      @Override
      public boolean isRelative()
      {
         return isRelative;
      }

      @Override
      public boolean isAnalog()
      {
         return isAnalog;
      }

      @Override
      public float getPollData()
      {
         return pollData;
      }

      @Override
      public String getName()
      {
         return name;
      }

      @Override
      public Identifier getIdentifier()
      {
         return identifier;
      }

      @Override
      public float getDeadZone()
      {
         return deadZone;
      }

      public void setRelative(boolean isRelative)
      {
         this.isRelative = isRelative;
      }

      public void setAnalog(boolean isAnalog)
      {
         this.isAnalog = isAnalog;
      }

      public void setPollData(float pollData)
      {
         this.pollData = pollData;
      }

      public void setName(String name)
      {
         this.name = name;
      }

      public void setIdentifier(Identifier identifier)
      {
         this.identifier = identifier;
      }

      public void setDeadZone(float deadZone)
      {
         this.deadZone = deadZone;
      }
   }
}
