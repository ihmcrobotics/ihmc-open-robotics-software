package us.ihmc.avatar.ros;

import handle_msgs.HandleControl;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;

public class HandDesiredConfigurationMessageToHandleControlMessageConverter
{
   public static void convertHandDesiredConfigurationMessage(HandDesiredConfigurationMessage ihmcMessage, HandleControl message)
   {
      switch (ihmcMessage.getHandDesiredConfiguration())
      {
         case CLOSE :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {40000, 40000, 40000, 0, 0});
            message.setValid(new boolean[] {true, true, true, true, true});

            break;

         case CLOSE_FINGERS :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {40000, 40000, 0, 0, 0});
            message.setValid(new boolean[] {true, true, false, false, true});

            break;

         case CLOSE_THUMB :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {0, 0, 40000, 0, 0});
            message.setValid(new boolean[] {false, false, true, true, false});

            break;

         case CRUSH :
            message.setType(new int[] {1, 1, 1, 1, 1});
            message.setValue(new int[] {100000, 100000, 30000, 0, 0});
            message.setValid(new boolean[] {true, true, true, true, true});

            break;

         case CRUSH_INDEX :
            message.setType(new int[] {1, 1, 1, 1, 1});
            message.setValue(new int[] {100000, 0, 0, 0, 0});
            message.setValid(new boolean[] {true, false, false, false, false});

            break;

         case CRUSH_MIDDLE :
            message.setType(new int[] {1, 1, 1, 1, 1});
            message.setValue(new int[] {0, 100000, 0, 0, 0});
            message.setValid(new boolean[] {false, true, false, false, false});

            break;

         case CRUSH_THUMB :
            message.setType(new int[] {1, 1, 1, 1, 1});
            message.setValue(new int[] {0, 0, 30000, 0, 0});
            message.setValid(new boolean[] {false, false, true, false, false});

            break;

         case HALF_CLOSE :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {15000, 15000, 15000, 0, 0});
            message.setValid(new boolean[] {true, true, true, true, true});

            break;

         case OPEN :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {0, 0, 0, 0, 0});
            message.setValid(new boolean[] {true, true, true, true, true});

            break;

         case OPEN_INDEX :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {0, 0, 0, 0, 0});
            message.setValid(new boolean[] {true, false, false, false, false});

            break;

         case OPEN_MIDDLE :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {0, 0, 0, 0, 0});
            message.setValid(new boolean[] {false, true, false, false, false});

            break;

         case OPEN_THUMB :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {0, 0, 0, 0, 0});
            message.setValid(new boolean[] {false, false, true, true, false});

            break;

         case SLOW_CLOSE :

         // TODO need to fix this
         // currently iRobot hand physics in Gazebo are weird causing SLOW_CLOSE and STOP to be kinda the same thing
         case STOP :
            message.setType(new int[] {1, 1, 1, 1, 1});
            message.setValue(new int[] {7000, 7000, 8500, 0, 0});
            message.setValid(new boolean[] {true, true, true, true, true});

            break;

         case T_SPREAD :
            message.setType(new int[] {2, 2, 2, 2, 2});
            message.setValue(new int[] {0, 0, 0, 0, 4000});
            message.setValid(new boolean[] {true, true, true, true, true});

            break;

         default :
            break;

      }
   }
}
