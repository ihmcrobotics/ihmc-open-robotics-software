package us.ihmc.avatar.networkProcessor.poseValidityToolboxModule;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class PoseValidityToolboxController extends ToolboxController
{
   public PoseValidityToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
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

}
