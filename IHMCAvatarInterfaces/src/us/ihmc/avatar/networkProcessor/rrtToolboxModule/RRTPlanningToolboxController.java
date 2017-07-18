package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RRTPlanningToolboxController extends ToolboxController
{

   public RRTPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      // TODO Auto-generated constructor stub
   }

   @Override
   protected void updateInternal()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   protected boolean initialize()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   protected boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

   public PacketConsumer createRequestConsumer()
   {
      // TODO Auto-generated method stub
      return null;
   }

}
