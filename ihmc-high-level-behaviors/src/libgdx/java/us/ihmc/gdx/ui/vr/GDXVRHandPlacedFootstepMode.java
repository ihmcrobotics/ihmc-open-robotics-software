package us.ihmc.gdx.ui.vr;

import org.lwjgl.openvr.*;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.HashMap;

public class GDXVRHandPlacedFootstepMode
{
   public void processVRInput(GDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side, controller ->
         {
//            VRControllerState controllerState = controller.getControllerState();
//            VRControllerAxis vrControllerAxis = controllerState.rAxis(2);
//            LogTools.info("axis: {} {}", vrControllerAxis.x(), vrControllerAxis.y());
////            LogTools.info("button pressed {}", controllerState.ulButtonPressed());
////            LogTools.info("button touched {}", controllerState.ulButtonTouched());
//
//
//
//
//
//
//            //            controller.isButtonNewlyPressed(GDXVRControllerAxes.SteamVR_Trigger)
//            HashMap<Integer, VREvent> events = vrContext.getDeviceIndexToEventsThisFrameMap().get(controller.getDeviceIndex());
//
//            if (events != null)
//            {
//               VREvent buttonPress = events.get(VR.EVREventType_VREvent_ButtonPress);
//               if (buttonPress != null)
//               {
//                  LogTools.info("{} button press {}", side, buttonPress.data().controller().button());
//               }
//               VREvent dualAnalogPress = events.get(VR.EVREventType_VREvent_DualAnalog_Press);
//               if (dualAnalogPress != null)
//               {
//                  LogTools.info("{} dual analog {}, {}", side, dualAnalogPress.data().dualAnalog().x(),
//                                dualAnalogPress.data().dualAnalog().y());
//               }
//            }
         });
      }

   }
}
