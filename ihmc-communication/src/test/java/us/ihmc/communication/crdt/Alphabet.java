package us.ihmc.communication.crdt;

import us.ihmc.communication.DataModified;

public class Alphabet implements LatestModificationReplicate<Alphabet>
{
   private final DataModified dataModified = new DataModified();
   private String alphabet = "";

   public void addLetter(String letter)
   {
      alphabet += letter;
      dataModified.markModified();
   }

   @Override
   public boolean getDataIsModified()
   {
      return dataModified.getDataIsModified();
   }

   @Override
   public void clearDataModified()
   {
      dataModified.clearModified();
   }

   @Override
   public void markDataModified()
   {
      dataModified.markModified();
   }

   public void setAlphabet(String alphabet)
   {
      this.alphabet = alphabet;
   }

   public String getAlphabet()
   {
      return alphabet;
   }
}
