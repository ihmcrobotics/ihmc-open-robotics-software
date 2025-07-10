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
import us.ihmc.perception.heightMap.HeightMapParameters;
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
   RDXROS2HeightMapVisualizer heightMapVisualizer;

   private final HeightMapMessage heightMapMessage = new HeightMapMessage();

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
                                    "JSON files (*.json){.json},All Files (*){.*}",
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
            System.out.println("Selected JSON file: " + selectedFile);

            ObjectMapper mapper = new ObjectMapper();
            mapper.configure(JsonParser.Feature.ALLOW_COMMENTS, true);

            // Read JSON file
            try
            {
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
            // Needs to be greater than 1
            heightMapMessage.setSequenceId(2);
            heightMapVisualizer.acceptHeightMapMessage(heightMapMessage);
         }

         ImGuiFileDialog.close();
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
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // Nothing happening in here yet
   }
}
