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
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.imgui.RDXPanel;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

public class RDXHeightMapPanel extends RDXPanel implements RenderableProvider
{

   private final HeightMapMessage heightMapMessage = new HeightMapMessage();

   public RDXHeightMapPanel()
   {
      super("Height Map Panel");
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void renderImGuiWidgets()
   {
      // Need this stuff to be in a panel
      ImGui.text("FUCK OFF");
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

      if (ImGuiFileDialog.display("chooseJsonFile", 0, 400, 200, 4000, 2000))
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

            Mat heightMapMat = HeightMapMessageTools.unpackMessageToMatColumnMajor(heightMapMessage, new HeightMapParameters());

            PerceptionDebugTools.printMat("s", heightMapMat, 5);
            heightMapMat.close();
         }
         ImGuiFileDialog.close();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }
}
