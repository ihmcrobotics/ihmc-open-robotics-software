package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class TerrainMapCommand implements Command<TerrainMapCommand, TerrainMapMessage>
{
   private long sequenceId;
   private final Point2D gridCenter = new Point2D();
   private double gridWidth;
   private double cellSize;
   private int cellsPerAxis;

   private final TDoubleArrayList heightMap = new TDoubleArrayList();
   private final TDoubleArrayList traversabilityMap = new TDoubleArrayList();
   private final RecyclingArrayList<Vector3D> normalMap = new RecyclingArrayList<>(Vector3D.class);

   @Override
   public void clear()
   {
      heightMap.reset();
      traversabilityMap.reset();
      normalMap.clear();
   }

   @Override
   public void setFromMessage(TerrainMapMessage message)
   {
      sequenceId = message.getSequenceId();
      gridCenter.set(message.getGridCenterX(), message.getGridCenterY());
      gridWidth = message.getWidthInMeters();
      cellSize = message.getCellSizeInMeters();
      cellsPerAxis = message.getCellsPerAxis();

      for (int i = 0; i < message.getHeightMap().size(); i++)
      {
         heightMap.add(message.getHeightMap().get(i));
         traversabilityMap.add(message.getTraversabilityScore().get(i));

         Vector3D normal = normalMap.add();
         normal.setX(unpackByteAsFloat(message.getSnappedNormalXData().get(i), -1.0f, 1.0f));
         normal.setY(unpackByteAsFloat(message.getSnappedNormalYData().get(i), -1.0f, 1.0f));
         normal.setZ(unpackByteAsFloat(message.getSnappedNormalZData().get(i), 0.0f, 1.0f));
      }
   }

   public static float unpackByteAsFloat(byte val, float minValue, float maxValue)
   {
      return (float) (val & 0xFF) * (maxValue - minValue) / 255 + minValue;
   }

   @Override
   public Class<TerrainMapMessage> getMessageClass()
   {
      return TerrainMapMessage.class;
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
   public void set(TerrainMapCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      gridCenter.set(other.gridCenter);
      gridWidth = other.gridWidth;
      cellSize = other.cellSize;
      cellsPerAxis = other.cellsPerAxis;

      heightMap.addAll(other.heightMap);
      traversabilityMap.addAll(other.traversabilityMap);

      for (int i = 0; i < other.normalMap.size(); i++)
      {
         normalMap.add().set(other.normalMap.get(i));
      }
   }
}
