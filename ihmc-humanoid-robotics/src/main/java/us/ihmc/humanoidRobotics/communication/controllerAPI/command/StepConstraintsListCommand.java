package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.StepConstraintsListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;

public class StepConstraintsListCommand implements Command<StepConstraintsListCommand, StepConstraintsListMessage>
{
   private final RecyclingArrayList<StepConstraintRegionCommand> stepsConstraints = new RecyclingArrayList<>(StepConstraintRegionCommand::new);

   public StepConstraintsListCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      stepsConstraints.clear();
   }

   @Override
   public void setFromMessage(StepConstraintsListMessage message)
   {
      clear();

      int upperBound = 0;
      int vertexIndex = 0;
      int convexPolygonIndexStart = 0;

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();

      for (int regionIndex = 0; regionIndex < message.getRegionOrigin().size(); regionIndex++)
      {
         StepConstraintRegionCommand planarRegionCommand = stepsConstraints.add();
         Point3D origin = message.getRegionOrigin().get(regionIndex);
         Vector3D normal = message.getRegionNormal().get(regionIndex);
         planarRegionCommand.setRegionProperties(origin, normal);

         upperBound += message.getConcaveHullsSize().get(regionIndex);

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            planarRegionCommand.addConcaveHullVertex().set(vertexBuffer.get(vertexIndex));
         }

         int polygonIndex = 0;

         for (; polygonIndex < message.getNumberOfHolesInRegion().get(regionIndex); polygonIndex++)
         {
            upperBound += message.getHolePolygonsSize().get(convexPolygonIndexStart + polygonIndex);
            ConcavePolygon2D convexPolygon = planarRegionCommand.addHoleInRegion();

            for (; vertexIndex < upperBound; vertexIndex++)
               convexPolygon.addVertex(vertexBuffer.get(vertexIndex));

            convexPolygon.update();
         }
         convexPolygonIndexStart += polygonIndex;
      }
   }

   @Override
   public void set(StepConstraintsListCommand command)
   {
      clear();
      for (int i = 0; i < command.stepsConstraints.size(); i++)
         this.stepsConstraints.add().set(command.stepsConstraints.get(i));
   }

   
   @Override
   public Class<StepConstraintsListMessage> getMessageClass()
   {
      return StepConstraintsListMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !stepsConstraints.isEmpty() && (stepsConstraints.get(0).isCommandValid());
   }

   public int getNumberOfConstraints()
   {
      return stepsConstraints.size();
   }

   public StepConstraintRegionCommand getStepConstraint(int index)
   {
      return stepsConstraints.get(index);
   }

   @Override
   public long getSequenceId()
   {
      return -1;
   }
}
