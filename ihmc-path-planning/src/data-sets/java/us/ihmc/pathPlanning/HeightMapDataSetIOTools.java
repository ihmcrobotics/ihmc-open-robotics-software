package us.ihmc.pathPlanning;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import perception_msgs.TerrainMapMessage;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;

import java.io.IOException;
import java.io.InputStream;

/**
 * Loads legacy height-map dataset JSON (serialized with {@code JSONSerializer} + PubSubType) into jros2 types.
 */
public class HeightMapDataSetIOTools
{
   private static final String LEGACY_HEIGHT_MAP_MESSAGE_KEY = "perception_msgs::msg::dds_::HeightMapMessage_";
   private static final double DEFAULT_MAP_WIDTH_METERS = 5.0;

   private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();

   public static TerrainMapData loadTerrainMapData(InputStream inputStream) throws IOException
   {
      JsonNode root = OBJECT_MAPPER.readTree(inputStream);
      JsonNode messageNode = root.get(LEGACY_HEIGHT_MAP_MESSAGE_KEY);
      if (messageNode == null && root.size() == 1)
         messageNode = root.elements().next();

      double gridCenterX = messageNode.get("grid_center_x").asDouble();
      double gridCenterY = messageNode.get("grid_center_y").asDouble();
      double cellSize = messageNode.get("cell_size_in_meters").asDouble();
      double mapWidth = messageNode.has("width_in_meters") ? messageNode.get("width_in_meters").asDouble() : DEFAULT_MAP_WIDTH_METERS;

      TerrainMapData terrainMapData = new TerrainMapData(cellSize, mapWidth, gridCenterX, gridCenterY);
      float[] heightMap = terrainMapData.getHeightMap();

      JsonNode keysNode = messageNode.get("keys");
      JsonNode heightsNode = messageNode.get("heights");
      if (keysNode != null && heightsNode != null)
      {
         for (int i = 0; i < keysNode.size(); i++)
         {
            int key = keysNode.get(i).asInt();
            heightMap[key] = (float) heightsNode.get(i).asDouble();
         }
      }

      return terrainMapData;
   }

   public static TerrainMapMessage loadTerrainMapMessage(InputStream inputStream) throws IOException
   {
      TerrainMapData terrainMapData = loadTerrainMapData(inputStream);
      TerrainMapMessage message = new TerrainMapMessage();
      TerrainMapMessageTools.toMessage(terrainMapData, message);
      return message;
   }

   private HeightMapDataSetIOTools()
   {
   }
}
