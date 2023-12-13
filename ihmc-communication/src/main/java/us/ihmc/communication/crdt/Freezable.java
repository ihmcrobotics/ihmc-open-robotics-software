package us.ihmc.communication.crdt;

/**
 * Part of a simple CRDT algorithm where something is frozen when modified
 * to allow the changes to propagate without being immediately overritten
 * with out of date data.
 */
public interface Freezable
{
   void freeze();

   void unfreeze();

   boolean isFrozen();
}
