package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class PlaneContactStateCommandPool implements InverseDynamicsCommand<PlaneContactStateCommandPool>
{
   private final RecyclingArrayList<PlaneContactStateCommand> pool = new RecyclingArrayList<>(8, PlaneContactStateCommand.class);

   public PlaneContactStateCommandPool()
   {
      clear();
   }

   public PlaneContactStateCommand createCommand()
   {
      return pool.add();
   }

   public PlaneContactStateCommand getCommand(int index)
   {
      return pool.get(index);
   }

   public int getNumberOfCommands()
   {
      return pool.size();
   }

   public void clear()
   {
      pool.clear();
   }

   @Override
   public void set(PlaneContactStateCommandPool other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         pool.add().set(other.getCommand(i));
   }

   @Override
   public InverseDynamicsCommandType getCommandType()
   {
      return InverseDynamicsCommandType.PLANE_CONTACT_STATE_POOL;
   }

   @Override
   public String toString()
   {
      return pool.toString();
   }
}
