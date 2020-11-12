package us.ihmc.avatar.ros.networkTest;

import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.yoVariables.registry.YoRegistry;

class ROS2NetworkTestYoVariablesUpdatedListener implements YoVariablesUpdatedListener
{
   private YoVariableClientInterface yoVariableClientInterface;
   private YoRegistry parentRegistry;
   private YoRegistry yoRegistry;

   public ROS2NetworkTestYoVariablesUpdatedListener(YoRegistry parentRegistry)
   {

      this.parentRegistry = parentRegistry;
   }

   @Override
   public boolean updateYoVariables()
   {
      return false;
   }

   @Override
   public boolean changesVariables()
   {
      return false;
   }

   @Override
   public void start(YoVariableClientInterface yoVariableClientInterface,
                     LogHandshake handshake,
                     YoVariableHandshakeParser handshakeParser,
                     DebugRegistry debugRegistry)
   {
      this.yoVariableClientInterface = yoVariableClientInterface;

      yoRegistry = handshakeParser.getRootRegistry();
      parentRegistry.addChild(yoRegistry);
   }

   @Override
   public void disconnected()
   {

   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {

   }

   @Override
   public void connected()
   {

   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {

   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {

   }

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {

   }
}
