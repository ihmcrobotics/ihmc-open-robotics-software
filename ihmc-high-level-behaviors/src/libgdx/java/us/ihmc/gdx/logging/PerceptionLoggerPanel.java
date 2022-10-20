package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.perception.HDF5Tools;
import us.ihmc.perception.OpenCLFloatBuffer;

import java.util.ArrayList;

public class PerceptionLoggerPanel extends ImGuiPanel
{

   private PerceptionDataLogger logger;
   private PerceptionDataLoader loader;
   private GDXPointCloudRenderer pointCloudRenderer;

   private final RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(200000, Point3D32::new);
   private ArrayList<String> topicNames = new ArrayList<>();
   private ArrayList<Integer> topicObjectCounts = new ArrayList<>();

   private ImInt topicIndex = new ImInt(0);
   private ImInt objectIndex = new ImInt(0);

   static final String PERCEPTION_LOG_FILE = "/home/bmishra/Workspace/Data/Sensor_Logs/experimental.hdf5";

   private String currentTopic;

   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionLoggerPanel(String panelName)
   {
      super(panelName);

      pointCloudRenderer = new GDXPointCloudRenderer();

      pointCloudRenderer.create(400000);
//      baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, GDXSceneLevel.MODEL);

      loader = new PerceptionDataLoader(PERCEPTION_LOG_FILE);
      topicNames = HDF5Tools.getTopicNames(loader.getHDF5Manager().getFile());
      for (String topic : topicNames)
      {
         topicObjectCounts.add((int) loader.getHDF5Manager().getCount(topic));
      }
      setRenderMethod(this::renderImguiWidgets);
   }

   public void renderImguiWidgets()
   {
      ImGui.text("Perception Log: " + loader.getFilePath());
      String[] names = topicNames.toArray(new String[0]);
      ImGui.combo("Topic Names", topicIndex, names);
      currentTopic = topicNames.get(topicIndex.get());
      ImGui.text("Total Files: " + topicObjectCounts.get(topicIndex.get()));

      ImGui.sliderInt("Object Index", objectIndex.getData(), 0, 10);
      if(ImGui.button("Load Next"))
      {
         loader.loadCompressedPointCloud("/os_cloud_node/points", objectIndex.get(), points);
      }
   }

   public void loadPointCloud()
   {
//      FusedSensorHeadPointCloudMessage fusedMessage = latestFusedSensorHeadPointCloudMessageReference.getAndSet(null);
//      if (fusedMessage != null)
//      {
//         decompressionInputDirectBuffer.rewind();
//         int numberOfBytes = fusedMessage.getScan().size();
//         decompressionInputDirectBuffer.limit(numberOfBytes);
//         for (int i = 0; i < numberOfBytes; i++)
//         {
//            decompressionInputDirectBuffer.put(fusedMessage.getScan().get(i));
//         }
//         decompressionInputDirectBuffer.flip();
//         decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
//         // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
//         lz4Decompressor.decompress(decompressionInputDirectBuffer, decompressedOpenCLIntBuffer.getBackingDirectByteBuffer());
//         decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
//
//         latestSegmentIndex = (int) fusedMessage.getSegmentIndex();
//
//         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, latestSegmentIndex);
//         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, pointSize.get());
//         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);
//
//         parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
//         decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);
//
//         openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
//         openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
//         openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
//         openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
//         pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
//
//         pointCloudRenderer.updateMeshFastest(totalNumberOfPoints);
//      }
   }
}
