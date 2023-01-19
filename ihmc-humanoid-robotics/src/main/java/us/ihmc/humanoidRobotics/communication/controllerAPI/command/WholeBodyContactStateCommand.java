package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.WholeBodyContactStateMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;

public class WholeBodyContactStateCommand implements Command<WholeBodyContactStateCommand, WholeBodyContactStateMessage>
{
   private long sequenceId = 0;
   private final TIntArrayList contactingBodiesHashCodes = new TIntArrayList(10);
   private final RecyclingArrayList<Point3D> contactPointPositions = new RecyclingArrayList<>(10, Point3D::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      contactingBodiesHashCodes.reset();
      contactPointPositions.clear();
   }

   @Override
   public void setFromMessage(WholeBodyContactStateMessage message)
   {
      this.sequenceId = message.getSequenceId();

      this.contactingBodiesHashCodes.reset();
      this.contactPointPositions.clear();

      for (int i = 0; i < message.getContactingBodiesHashCodes().size(); i++)
      {
         this.contactingBodiesHashCodes.add(message.getContactingBodiesHashCodes().get(i));
         this.contactPointPositions.add().set(message.getContactPointPositions().get(i));
      }
   }

   @Override
   public Class<WholeBodyContactStateMessage> getMessageClass()
   {
      return WholeBodyContactStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(WholeBodyContactStateCommand other)
   {
      this.sequenceId = other.sequenceId;

      this.contactingBodiesHashCodes.reset();
      this.contactPointPositions.clear();

      for (int i = 0; i < other.getContactingBodiesHashCodes().size(); i++)
      {
         this.contactingBodiesHashCodes.add(other.contactingBodiesHashCodes.get(i));
         this.contactPointPositions.add().set(other.contactPointPositions.get(i));
      }
   }

   public TIntArrayList getContactingBodiesHashCodes()
   {
      return contactingBodiesHashCodes;
   }

   public RecyclingArrayList<Point3D> getContactPointPositions()
   {
      return contactPointPositions;
   }
}
