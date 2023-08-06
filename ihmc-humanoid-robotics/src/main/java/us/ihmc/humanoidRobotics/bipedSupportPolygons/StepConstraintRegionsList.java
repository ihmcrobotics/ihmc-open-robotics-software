package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import controller_msgs.msg.dds.StepConstraintsListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintRegionCommand;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

import java.util.List;
import java.util.Vector;

public class StepConstraintRegionsList
{
   private final RecyclingArrayList<StepConstraintRegion> stepConstraintRegions = new RecyclingArrayList<>(StepConstraintRegion::new);

   public void clear()
   {
      stepConstraintRegions.clear();
   }

   public StepConstraintRegion getNextConstraintRegion()
   {
      StepConstraintRegion region = stepConstraintRegions.add();
      region.clear();
      return region;
   }

   public void set(StepConstraintRegionsList other)
   {
      stepConstraintRegions.clear();
      for (int i = 0; i < other.stepConstraintRegions.size(); i++)
      {
         stepConstraintRegions.add().set(other.stepConstraintRegions.get(i));
      }
   }

   public List<StepConstraintRegion> getAsList()
   {
      return stepConstraintRegions;
   }

   public void addOffset(Vector3DReadOnly offset)
   {
      for (int i = 0; i < stepConstraintRegions.size(); i++)
      {
         stepConstraintRegions.get(i).addOffset(offset);
      }
   }

   public void getAsMessage(StepConstraintsListMessage message)
   {
      getAsMessage(stepConstraintRegions, message);
   }

   public static void getAsMessage(List<StepConstraintRegion> stepConstraintRegions, StepConstraintsListMessage message)
   {
      message.getVertexBuffer().clear();
      message.getRegionOrigin().clear();
      message.getRegionNormal().clear();
      message.getRegionOrientation().clear();
      message.getConcaveHullsSize().clear();
      message.getNumberOfHolesInRegion().clear();
      message.getHolePolygonsSize().clear();


      for (int regionIndex = 0; regionIndex < stepConstraintRegions.size(); regionIndex++)
      {
         StepConstraintRegion stepConstraintRegion = stepConstraintRegions.get(regionIndex);

         message.getRegionOrigin().add().set(stepConstraintRegion.getPoint());
         message.getRegionOrientation().add().set(stepConstraintRegion.getTransformToWorld().getRotation()); // TODO check this
         message.getRegionNormal().add().set(stepConstraintRegion.getNormal()); // TODO check this

         int concaveHullSize = stepConstraintRegion.getConcaveHullSize();
         message.getConcaveHullsSize().add(concaveHullSize);
         for (int vertexIndex = 0; vertexIndex < concaveHullSize; vertexIndex++)
            message.getVertexBuffer().add().set(stepConstraintRegion.getConcaveHullVertices().get(vertexIndex));

         int numberOfHoles = stepConstraintRegion.getNumberOfHolesInRegion();
         message.getNumberOfHolesInRegion().add(numberOfHoles);
         for (int holeIndex = 0; holeIndex < numberOfHoles; holeIndex++)
         {
            ConcavePolygon2DReadOnly hole = stepConstraintRegion.getHoleInConstraintRegion(holeIndex);
            int holeVertices = hole.getNumberOfVertices();
            message.getHolePolygonsSize().add(holeVertices);
            for (int holeVertexIndex = 0; holeVertexIndex < holeVertices; holeVertexIndex++)
               message.getVertexBuffer().add().set(hole.getVertex(holeVertexIndex));
         }
      }
   }
}
