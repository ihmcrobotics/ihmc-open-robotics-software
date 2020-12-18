package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputProcessor;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * TODO delete
 */
public class GDXInputMapperHelper implements InputProcessor
{
   ArrayList<String> inputNames = new ArrayList<>();
   HashMap<String, ActionEvent> actionMappings = new HashMap<>();
   HashMap<String, AnalogEvent> analogMappings = new HashMap<>();

   public interface ActionEvent
   {
      void onAction(boolean isPressed, float timePerFrame);
   }

   public interface AnalogEvent
   {
      void onAnalog(float value, float timePerFrame);
   }

   public GDXInputMapperHelper()
   {
      Gdx.input.setInputProcessor(this);
   }

//   public void addActionMapping(String name, Trigger trigger, ActionEvent actionEvent)
//   {
//      inputNames.add(name);
//      inputManager.addMapping(name, trigger);
//      actionMappings.put(name, actionEvent);
//   }
//
//   public void addAnalogMapping(String name, Trigger trigger, AnalogEvent actionEvent)
//   {
//      inputNames.add(name);
//      inputManager.addMapping(name, trigger);
//      analogMappings.put(name, actionEvent);
//   }
//
//   public void build()
//   {
//      inputManager.addListener(this, inputNames.toArray(new String[0]));
//   }

//   @Override
//   public void onAction(String name, boolean isPressed, float tpf)
//   {
////      System.out.println(name + " isPressed " + isPressed + " tpf " + tpf);
//      ActionEvent actionEvent = actionMappings.get(name);
//      if (actionEvent != null)
//      {
//         actionEvent.onAction(isPressed, tpf);
//      }
//   }
//
//   @Override
//   public void onAnalog(String name, float value, float tpf)
//   {
////      System.out.println(name + " value " + value + " tpf " + tpf);
//      AnalogEvent analogEvent = analogMappings.get(name);
//      if (analogEvent != null)
//      {
//         analogEvent.onAnalog(value, tpf);
//      }
//   }

   @Override
   public boolean keyDown(int keycode)
   {
      return false;
   }

   @Override
   public boolean keyUp(int keycode)
   {
      return false;
   }

   @Override
   public boolean keyTyped(char character)
   {
      return false;
   }

   @Override
   public boolean touchDown(int screenX, int screenY, int pointer, int button)
   {
      return false;
   }

   @Override
   public boolean touchUp(int screenX, int screenY, int pointer, int button)
   {
      return false;
   }

   @Override
   public boolean touchDragged(int screenX, int screenY, int pointer)
   {
      return false;
   }

   @Override
   public boolean mouseMoved(int screenX, int screenY)
   {
      return false;
   }

   @Override
   public boolean scrolled(float amountX, float amountY)
   {
      return false;
   }
}