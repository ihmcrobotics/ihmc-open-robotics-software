package us.ihmc.perception.lerobot;

import us.ihmc.commons.thread.RepeatingTaskThread;

public class LeRobotManager
{
   private final String modelName;

   private final RepeatingTaskThread thread = new RepeatingTaskThread("LeRobotROS2Thread", this::update);

   public LeRobotManager(String modelName)
   {
      this.modelName = modelName;
   }

   private void update()
   {

   }

   public void setRunning(boolean running)
   {
      if (running)
         thread.startRepeating();
      else
         thread.stopRepeating();
   }

   public void destroy()
   {
      thread.kill();
   }

   public String getModelName()
   {
      return modelName;
   }
}
