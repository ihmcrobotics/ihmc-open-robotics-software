package us.ihmc.rdx.ui.hands;

import us.ihmc.robotics.robotSide.RobotSide;

public interface RDXHandInterface
{
   enum HandAction
   {
      OPEN, CLOSE, GRIP, CALIBRATE, RESET
   }

   RobotSide getSide();

   boolean isCalibrated();

   boolean needsReset();

   void update();

   void renderImGuiWidgets();

   void sendCommand(HandAction action);

   void sendFingerPosition(int index, float value);

   float getFingerPosition(int index);
}
