package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence.Object;

public class PlanarRegionsListCommand implements Command<PlanarRegionsListCommand, PlanarRegionsListMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<PlanarRegionCommand> planarRegions = new RecyclingArrayList<>(30, PlanarRegionCommand.class);

   public PlanarRegionsListCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      planarRegions.clear();
   }

   @Override
   public void setFromMessage(PlanarRegionsListMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();

      int upperBound = 0;
      int vertexIndex = 0;
      int convexPolygonIndexStart = 0;

      Object<Point3D> vertexBuffer = message.getVertexBuffer();

      for (int regionIndex = 0; regionIndex < message.getRegionId().size(); regionIndex++)
      {
         PlanarRegionCommand planarRegionCommand = planarRegions.add();
         int regionId = message.getRegionId().get(regionIndex);
         Point3D origin = message.getRegionOrigin().get(regionIndex);
         Vector3D normal = message.getRegionNormal().get(regionIndex);
         planarRegionCommand.setRegionProperties(regionId, origin, normal);

         upperBound += message.getConcaveHullsSize().get(regionIndex);

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            planarRegionCommand.addConcaveHullVertex().set(vertexBuffer.get(vertexIndex));
         }

         int polygonIndex = 0;

         for (; polygonIndex < message.getNumberOfConvexPolygons().get(regionIndex); polygonIndex++)
         {
            upperBound += message.getConvexPolygonsSize().get(convexPolygonIndexStart + polygonIndex);
            ConvexPolygon2D convexPolygon = planarRegionCommand.addConvexPolygon();

            for (; vertexIndex < upperBound; vertexIndex++)
               convexPolygon.addVertex(vertexBuffer.get(vertexIndex));

            convexPolygon.update();
         }
         convexPolygonIndexStart += polygonIndex;
      }
   }

   @Override
   public void set(PlanarRegionsListCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      RecyclingArrayList<PlanarRegionCommand> dataList = other.getPlanarRegions();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            planarRegions.add().set(dataList.get(i));
      }
   }

   public int getNumberOfPlanarRegions()
   {
      return planarRegions.size();
   }

   public RecyclingArrayList<PlanarRegionCommand> getPlanarRegions()
   {
      return planarRegions;
   }

   public PlanarRegionCommand getPlanarRegionCommand(int i)
   {
      return planarRegions.get(i);
   }

   @Override
   public Class<PlanarRegionsListMessage> getMessageClass()
   {
      return PlanarRegionsListMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfPlanarRegions() > 0;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
