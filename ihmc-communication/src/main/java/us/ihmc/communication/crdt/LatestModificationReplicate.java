package us.ihmc.communication.crdt;

public interface LatestModificationReplicate<T>
{
   boolean getDataIsModified();

   void clearDataModified();

   void markDataModified();
}
