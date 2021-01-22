package us.ihmc.robotDataVisualizer;

import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BasicYoVariablesUpdatedListener implements YoVariablesUpdatedListener
{
   private YoVariableClientInterface yoVariableClientInterface;
   private YoRegistry parentRegistry;
   private YoRegistry clientRootRegistry;
   private boolean handshakeComplete;

   public BasicYoVariablesUpdatedListener(YoRegistry parentRegistry)
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

      clientRootRegistry = handshakeParser.getRootRegistry();
      YoRegistry serverRegistry = new YoRegistry(yoVariableClientInterface.getServerName() + "Container");
      serverRegistry.addChild(clientRootRegistry);
      parentRegistry.addChild(serverRegistry);

      handshakeComplete = true;
   }

   public boolean isHandshakeComplete()
   {
      return handshakeComplete;
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
