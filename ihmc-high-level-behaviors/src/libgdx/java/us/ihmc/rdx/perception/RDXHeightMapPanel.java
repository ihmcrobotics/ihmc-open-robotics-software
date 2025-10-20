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
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuMapping.HeightMapLogReader;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Set;

public class RDXHeightMapPanel extends RDXPanel implements RenderableProvider
{
   private final RDXROS2HeightMapVisualizer heightMapVisualizer;

   private final HeightMapMessage heightMapMessage = new HeightMapMessage();
   private HeightMapLogReader logReader;
   private int currentFrameIndex = 0;

   private boolean isPlaying = false;
   private final float[] playbackSpeedSeconds = new float[] {0.1f};
   private long lastPlaybackTime = System.nanoTime();
   private float currentPlayBackSpeed;

   public RDXHeightMapPanel()
   {
      super("Height Map Panel");
      setRenderMethod(this::renderImGuiWidgets);

      heightMapVisualizer = new RDXROS2HeightMapVisualizer("Height Map Visualizer");
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
            LogTools.info("Selected file: " + selectedFile);

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
                  logReader = new HeightMapLogReader(selectedFile);
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
         int[] frameIndexArray = {currentFrameIndex};
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

         // Playback speed slider
         ImGui.sliderFloat("Playback Speed (sec/frame)", playbackSpeedSeconds, 0.033f, 1.0f, "%.4f");
         currentPlayBackSpeed = playbackSpeedSeconds[0];

         // Play / Pause button
         if (ImGui.button(isPlaying ? "Pause" : "Play"))
         {
            isPlaying = !isPlaying;
            lastPlaybackTime = System.nanoTime(); // reset timer on play
         }
      }

      // Playback logic
      if (isPlaying && logReader != null)
      {
         long now = System.nanoTime();
         float elapsedSeconds = (now - lastPlaybackTime) / 1e9f;

         if (elapsedSeconds >= currentPlayBackSpeed)
         {
            lastPlaybackTime = now;

            if (currentFrameIndex < logReader.getFrameCount() - 1)
            {
               currentFrameIndex++;

               try
               {
                  HeightMapMessage msg = logReader.loadFrame(currentFrameIndex);
                  if (msg != null)
                  {
                     heightMapMessage.set(msg);
                     heightMapVisualizer.acceptHeightMapMessage(heightMapMessage);
                  }
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
            else
            {
               // Reached the end, stop playing
               isPlaying = false;
            }
         }
      }

      heightMapVisualizer.renderImGuiWidgets();
      heightMapVisualizer.update();
   }

   public void getRenderablesFull(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      heightMapVisualizer.getRenderables(renderables, pool, sceneLevels);
   }

   public void destroy()
   {
      heightMapVisualizer.destroy();
      try
      {
         logReader.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // Nothing happening in here yet
   }
}
