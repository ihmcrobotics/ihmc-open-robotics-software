package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import imgui.ImGui;
import imgui.extension.imguifiledialog.ImGuiFileDialog;
import imgui.extension.imguifiledialog.flag.ImGuiFileDialogFlags;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.perception.heightMap.HeightMapBinaryLogReader;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Set;

public class RDXHeightMapPanel extends RDXPanel implements RenderableProvider
{
   RDXROS2HeightMapVisualizer heightMapVisualizer;

   private final HeightMapMessage heightMapMessage = new HeightMapMessage();
   private HeightMapBinaryLogReader logReader;
   int currentFrameIndex = 0;

   public RDXHeightMapPanel()
   {
      super("Height Map Panel");
      setRenderMethod(this::renderImGuiWidgets);

      HeightMapParameters heightMapParameters = new HeightMapParameters();
      heightMapVisualizer = new RDXROS2HeightMapVisualizer("Height Map Visualizer", heightMapParameters);
      heightMapVisualizer.setActive(true);
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Select Directory"))
      {
         String logPath = System.getProperty("user.home");
         ImGuiFileDialog.openDialog("chooseJsonFile",
                                    "Select File",
                                    "Binary files (*.bin){.bin},JSON files (*.json){.json},All Files (*){.*}",
                                    logPath,
                                    "",
                                    ImGuiFileDialogFlags.ConfirmOverwrite,
                                    0,
                                    0);
      }

      if (ImGuiFileDialog.display("chooseJsonFile", 0, 800, 800, 4000, 2000))
      {
         if (ImGuiFileDialog.isOk())
         {
            String selectedFile = ImGuiFileDialog.getFilePathName();
            System.out.println("Selected file: " + selectedFile);

            if (selectedFile.endsWith(".json"))
            {
               try
               {
                  ObjectMapper mapper = new ObjectMapper();
                  mapper.configure(JsonParser.Feature.ALLOW_COMMENTS, true);

                  File file = new File(selectedFile);
                  InputStream requestPacketInputStream = new FileInputStream(file);
                  JsonNode jsonNode = mapper.readTree(requestPacketInputStream);
                  JSONSerializer<HeightMapMessage> heightMapSerializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
                  heightMapMessage.set(heightMapSerializer.deserialize(jsonNode.toString()));
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }

               // Because the visualizer doesn't visualize the first one, increment the sequence ID
               heightMapMessage.setSequenceId(2);
               heightMapVisualizer.acceptHeightMapMessage(heightMapMessage);
            }
            else if (selectedFile.endsWith(".bin"))
            {
               try
               {
                  logReader = new HeightMapBinaryLogReader(selectedFile);
                  System.out.println("Loaded binary log with " + logReader.getFrameCount() + " frames.");

                  currentFrameIndex = 0;

                  HeightMapMessage msg = logReader.loadFrame(currentFrameIndex);
                  heightMapMessage.set(msg);
                  heightMapVisualizer.acceptHeightMapMessage(heightMapMessage);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
            else
            {
               System.out.println("Unsupported file type: " + selectedFile);
            }
         }

         ImGuiFileDialog.close();
      }

      if (logReader != null)
      {
         int frameCount = logReader.getFrameCount();

         ImGui.text("Binary Height Map Log Loaded:");
         ImGui.sameLine();

         if (ImGui.button("|<"))
         {
            currentFrameIndex = 0;
         }
         ImGui.sameLine();

         if (ImGui.button("<"))
         {
            if (currentFrameIndex > 0)
               currentFrameIndex--;
         }
         ImGui.sameLine();

         ImGui.text("Frame: " + (currentFrameIndex + 1) + "/" + frameCount);
         ImGui.sameLine();

         if (ImGui.button(">"))
         {
            if (currentFrameIndex < frameCount - 1)
               currentFrameIndex++;
         }
         ImGui.sameLine();

         if (ImGui.button(">|"))
         {
            currentFrameIndex = frameCount - 1;
         }

         // Slider for direct frame selection
         int[] frameIndexArray = { currentFrameIndex };
         if (ImGui.sliderInt("Frame", frameIndexArray, 0, frameCount - 1))
         {
            currentFrameIndex = frameIndexArray[0];
         }

         if (ImGui.button("Load Frame"))
         {
            try
            {
               HeightMapMessage msg = logReader.loadFrame(currentFrameIndex);
               if (msg != null)
               {
                  heightMapMessage.set(msg);
                  heightMapVisualizer.acceptHeightMapMessage(heightMapMessage);
                  System.out.println("Loaded frame #" + (currentFrameIndex + 1));
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      }

      heightMapVisualizer.renderImGuiWidgets();
      heightMapVisualizer.update();
   }

   public static HeightMapMessage loadFirstHeightMapFromBinary(String path) throws IOException
   {
      try (FileInputStream fis = new FileInputStream(path))
      {
         byte[] intBytes = new byte[4];

         if (fis.read(intBytes) != 4)
            return null;

         ByteBuffer intBuffer = ByteBuffer.wrap(intBytes).order(ByteOrder.LITTLE_ENDIAN);
         int frameSize = intBuffer.getInt();

         byte[] frameBytes = new byte[frameSize];
         int bytesRead = fis.read(frameBytes);
         if (bytesRead != frameSize)
            throw new IOException("Unexpected EOF while reading frame payload");

         ByteBuffer frameBuffer = ByteBuffer.wrap(frameBytes).order(ByteOrder.LITTLE_ENDIAN);
         double timestamp = frameBuffer.getDouble();

         int numFloats = (frameSize - 8) / Float.BYTES;
         float[] packedArray = new float[numFloats];
         frameBuffer.asFloatBuffer().get(packedArray);

         // Unpack header
         float widthInMeters = packedArray[0];
         float cellSizeInMeters = packedArray[1];
         float centerX = packedArray[2];
         float centerY = packedArray[3];
         float heightOffset = packedArray[4];
         float heightScaleFactor = packedArray[5];

         final int headerFloats = 6;

         int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
         int cellsPerAxis = 2 * centerIndex + 1;
         int totalCells = cellsPerAxis * cellsPerAxis;

         float[] heightsArray = new float[totalCells];
         System.arraycopy(packedArray, headerFloats, heightsArray, 0, totalCells);

         Mat mat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
         FloatBuffer matBuffer = mat.createBuffer();
         matBuffer.put(heightsArray);
         matBuffer.rewind();

         Point3D center = new Point3D(centerX, centerY, 0.0);

         // Convert to HeightMapMessage
         HeightMapMessage msg = new HeightMapMessage();
         HeightMapMessageTools.toMessage(mat, msg, center, widthInMeters, cellSizeInMeters);

         System.out.println("Loaded frame from binary log. Timestamp = " + timestamp);

         return msg;
      }
   }

   public void getRenderablesFull(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      heightMapVisualizer.getRenderables(renderables, pool, sceneLevels);
   }

   public void destroy()
   {
      heightMapVisualizer.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // Nothing happening in here yet
   }
}
