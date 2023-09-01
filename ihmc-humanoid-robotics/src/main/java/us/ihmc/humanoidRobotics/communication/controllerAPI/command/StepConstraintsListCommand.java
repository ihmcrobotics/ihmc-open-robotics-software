package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.StepConstraintsListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegionsList;
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
      for (int i = 0; i < stepsConstraints.size(); i++)
         stepsConstraints.get(i).clear();
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
         Quaternion orientation = message.getRegionOrientation().get(regionIndex);
         planarRegionCommand.setRegionTransformProperties(origin, orientation);

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

   public void getAsMessage(StepConstraintsListMessage message)
   {
      message.getVertexBuffer().clear();
      message.getRegionOrigin().clear();
      message.getRegionNormal().clear();
      message.getRegionOrientation().clear();
      message.getConcaveHullsSize().clear();
      message.getNumberOfHolesInRegion().clear();
      message.getHolePolygonsSize().clear();

      for (int regionIndex = 0; regionIndex < stepsConstraints.size(); regionIndex++)
      {
         StepConstraintRegionCommand regionCommand = stepsConstraints.get(regionIndex);

         message.getRegionOrigin().add().set(regionCommand.getRegionOrigin());
         message.getRegionOrientation().add().set(regionCommand.getTransformToWorld().getRotation()); // TODO check this
         message.getRegionNormal().add().set(regionCommand.getRegionNormal());

         int concaveHullSize = regionCommand.getConcaveHullVertices().size();
         message.getConcaveHullsSize().add(concaveHullSize);
         for (int vertexIndex = 0; vertexIndex < concaveHullSize; vertexIndex++)
            message.getVertexBuffer().add().set(regionCommand.getConcaveHullVertices().get(vertexIndex));

         int numberOfHoles = regionCommand.getHolesInRegion().size();
         message.getNumberOfHolesInRegion().add(numberOfHoles);
         for (int holeIndex = 0; holeIndex < numberOfHoles; holeIndex++)
         {
            ConcavePolygon2D hole = regionCommand.getHolesInRegion().get(holeIndex);
            int holeVertices = hole.getNumberOfVertices();
            message.getHolePolygonsSize().add(holeVertices);
            for (int holeVertexIndex = 0; holeVertexIndex < holeVertices; holeVertexIndex++)
               message.getVertexBuffer().add().set(hole.getVertex(holeVertexIndex));
         }
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

   public void addOffset(Vector3DReadOnly offset)
   {
      for (int i = 0; i < stepsConstraints.size(); i++)
         stepsConstraints.get(i).addOffset(offset);
   }

   public void get(StepConstraintRegionsList stepConstraintRegionList)
   {
      stepConstraintRegionList.clear();
      for (int i = 0; i < stepsConstraints.size(); i++)
      {
         stepsConstraints.get(i).getStepConstraintRegion(stepConstraintRegionList.getNextConstraintRegion());
      }
   }

   @Override
   public long getSequenceId()
   {
      return -1;
   }
}
