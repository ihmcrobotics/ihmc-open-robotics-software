package us.ihmc.jMonkeyEngineToolkit.jme.input;

import com.jme3.input.KeyInput;

import us.ihmc.tools.inputDevices.keyboard.Key;

public class JMEModifierKey
{
   public static int fromModifierKey(Key key)
   {
      switch(key)
      {
         case ONE:  return KeyInput.KEY_1;
         case TWO:  return KeyInput.KEY_2;
         case THREE:  return KeyInput.KEY_3;
         case FOUR:  return KeyInput.KEY_4;
         case FIVE:  return KeyInput.KEY_5;
         case SIX:  return KeyInput.KEY_6;
         case SEVEN:  return KeyInput.KEY_7;
         case EIGHT:  return KeyInput.KEY_8;
         case NINE:  return KeyInput.KEY_9;
         case ZERO:  return KeyInput.KEY_0;
         case PLUS:  return KeyInput.KEY_ADD;
         case MINUS:  return KeyInput.KEY_MINUS;
         case LEFT: return KeyInput.KEY_LEFT;
         case RIGHT: return KeyInput.KEY_RIGHT;
         case UP: return KeyInput.KEY_UP;
         case DOWN: return KeyInput.KEY_DOWN;
         case SHIFT: return KeyInput.KEY_LSHIFT;
         case CTRL: return KeyInput.KEY_LCONTROL;
         case META: return KeyInput.KEY_LMETA;
         case ALT: return KeyInput.KEY_LMENU;
         case SPACE: return KeyInput.KEY_SPACE;
         case A: return KeyInput.KEY_A;
         case B: return KeyInput.KEY_B;
         case C: return KeyInput.KEY_C;
         case D: return KeyInput.KEY_D;
         case E: return KeyInput.KEY_E;
         case F: return KeyInput.KEY_F;
         case G: return KeyInput.KEY_G;
         case H: return KeyInput.KEY_H;
         case I: return KeyInput.KEY_I;
         case J: return KeyInput.KEY_J;
         case K: return KeyInput.KEY_K;
         case L: return KeyInput.KEY_L;
         case M: return KeyInput.KEY_M;
         case N: return KeyInput.KEY_N;
         case O: return KeyInput.KEY_O;
         case P: return KeyInput.KEY_P;
         case Q: return KeyInput.KEY_Q;
         case R: return KeyInput.KEY_R;
         case S: return KeyInput.KEY_S;
         case T: return KeyInput.KEY_T;
         case U: return KeyInput.KEY_U;
         case V: return KeyInput.KEY_V;
         case W: return KeyInput.KEY_W;
         case X: return KeyInput.KEY_X;
         case Y: return KeyInput.KEY_Y;
         case Z: return KeyInput.KEY_Z;
         default:
            throw new RuntimeException("Key " + key + " not defined");
      }
   }
}
