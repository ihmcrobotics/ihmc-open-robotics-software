package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

public interface CollidableHolder extends Iterable<Collidable>
{
   default int getNumberOfCollidables()
   {
      return getCollidables().size();
   }

   List<Collidable> getCollidables();

   @Override
   default Iterator<Collidable> iterator()
   {
      return getCollidables().iterator();
   }

   public static CollidableHolder fromCollection(Collection<Collidable> collidables)
   {
      List<Collidable> collidableList = new ArrayList<>(collidables);
      return () -> collidableList;
   }
}