package us.ihmc.jme;

import com.jme3.input.InputManager;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.Trigger;

import java.util.ArrayList;
import java.util.HashMap;

public class JMEInputMapperHelper implements ActionListener, AnalogListener
{
   ArrayList<String> inputNames = new ArrayList<>();
   HashMap<String, ActionEvent> actionMappings = new HashMap<>();
   HashMap<String, AnalogEvent> analogMappings = new HashMap<>();
   private InputManager inputManager;

   public interface ActionEvent
   {
      void onAction(boolean isPressed, float timePerFrame);
   }

   public interface AnalogEvent
   {
      void onAnalog(float value, float timePerFrame);
   }

   public JMEInputMapperHelper(InputManager inputManager)
   {
      this.inputManager = inputManager;
   }

   public void addActionMapping(String name, Trigger trigger, ActionEvent actionEvent)
   {
      inputNames.add(name);
      inputManager.addMapping(name, trigger);
      actionMappings.put(name, actionEvent);
   }

   public void addAnalogMapping(String name, Trigger trigger, AnalogEvent actionEvent)
   {
      inputNames.add(name);
      inputManager.addMapping(name, trigger);
      analogMappings.put(name, actionEvent);
   }

   public void build()
   {
      inputManager.addListener(this, inputNames.toArray(new String[0]));
   }

   @Override
   public void onAction(String name, boolean isPressed, float tpf)
   {
//      System.out.println(name + " isPressed " + isPressed + " tpf " + tpf);
      ActionEvent actionEvent = actionMappings.get(name);
      if (actionEvent != null)
      {
         actionEvent.onAction(isPressed, tpf);
      }
   }

   @Override
   public void onAnalog(String name, float value, float tpf)
   {
//      System.out.println(name + " value " + value + " tpf " + tpf);
      AnalogEvent analogEvent = analogMappings.get(name);
      if (analogEvent != null)
      {
         analogEvent.onAnalog(value, tpf);
      }
   }
}