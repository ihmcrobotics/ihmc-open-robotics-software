//package us.ihmc.gdx.ui.graphics.live;
//
//import com.badlogic.gdx.Gdx;
//import com.badlogic.gdx.graphics.*;
//import com.badlogic.gdx.graphics.g3d.Material;
//import com.badlogic.gdx.graphics.g3d.Renderable;
//import com.badlogic.gdx.graphics.g3d.RenderableProvider;
//import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
//import com.badlogic.gdx.graphics.g3d.particles.ParticleShader;
//import com.badlogic.gdx.graphics.glutils.ShaderProgram;
//import com.badlogic.gdx.utils.Array;
//import com.badlogic.gdx.utils.Pool;
//import imgui.internal.ImGui;
//import imgui.type.ImFloat;
//import sensor_msgs.PointCloud2;
//import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
//import us.ihmc.commons.lists.RecyclingArrayList;
//import us.ihmc.euclid.referenceFrame.ReferenceFrame;
//import us.ihmc.euclid.transform.RigidBodyTransform;
//import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
//import us.ihmc.euclid.tuple3D.Point3D32;
//import us.ihmc.gdx.GDXPointCloudRenderer;
//import us.ihmc.gdx.imgui.ImGuiPlot;
//import us.ihmc.log.LogTools;
//import us.ihmc.tools.thread.MissingThreadTools;
//import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
//import us.ihmc.utilities.ros.RosMainNode;
//import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
//
//public class temp implements RenderableProvider
//{
//   private static final int MAX_POINTS = 50000;
//
//   private final RosMainNode ros1Node;
//   private final String ros1PointCloudTopic;
//   private final ReferenceFrame sensorBaseFrame;
//   private final RigidBodyTransformReadOnly baseToSensorTransform;
//   private final RigidBodyTransform tempBaseToSensorTransform = new RigidBodyTransform();
//   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
//
//
//   private boolean packingA = true;
//   private final RecyclingArrayList<Point3D32> pointsA = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);
//   private final RecyclingArrayList<Point3D32> pointsB = new RecyclingArrayList<>(MAX_POINTS, Point3D32::new);
//
//   private final ResettableExceptionHandlingExecutorService threadQueue;
//
//   private GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
//   private final RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);
//
//   private boolean enabled = false;
//
//   private long receivedCount = 0;
//   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);
//
//   private Renderable renderable;
//   private float[] vertices;
//   //   private MeshBuilder newMesh;
//   private short[] indices;
//   private static final int SIZE_AND_ROTATION_USAGE = 1 << 9;
//
//   private final VertexAttributes vertexAttributes = new VertexAttributes(
//         new VertexAttribute(VertexAttributes.Usage.Position, 3, ShaderProgram.POSITION_ATTRIBUTE),
//         new VertexAttribute(VertexAttributes.Usage.ColorUnpacked, 4,ShaderProgram.COLOR_ATTRIBUTE),
//         new VertexAttribute(SIZE_AND_ROTATION_USAGE, 3, "a_sizeAndRotation")
//   );
//   private final int vertexSize = 10;
//
//
//   public GDXROS1PointCloudVisualizer(RosMainNode ros1Node,
//                                      String ros1PointCloudTopic,
//                                      ReferenceFrame sensorBaseFrame,
//                                      RigidBodyTransformReadOnly baseToSensorTransform)
//   {
//
//   }
//
//   public void create()
//   {
//
//      renderable = new Renderable();
//      renderable.meshPart.primitiveType = GL20.GL_POINTS;
//      renderable.meshPart.offset = 0;
//      renderable.material = new Material(ColorAttribute.createDiffuse(Color.RED));
//
//      vertices = new float[50000 * vertexSize];
//      indices = new short[50000 * vertexSize];
//      //      newMesh = new MeshBuilder();
//      //      newMesh.begin(vertexAttributes, GL20.GL_POINTS);
//
//      if (renderable.meshPart.mesh != null)
//         renderable.meshPart.mesh.dispose();
//      renderable.meshPart.mesh = new Mesh(true, 50000, 0, vertexAttributes);
//
//      ParticleShader.Config config = new ParticleShader.Config(ParticleShader.ParticleType.Point);
//      renderable.shader = new ParticleShader(renderable, config);
//      //      ((ParticleShader) renderable.shader).set(ShaderProgram.COLOR_ATTRIBUTE., Color.RED);
//      renderable.shader.init();
//   }
//
//   public void updateMeshcolor(float alpha)
//   {
//      if (enabled)
//      {
//         pointsToRender.clear();
//         synchronized (pointsToRender)
//         {
//            RecyclingArrayList<Point3D32> pointsToRead = packingA ? pointsB : pointsA;
//            for (Point3D32 point : pointsToRead)
//            {
//               pointsToRender.add().set(point);
//            }
//         }
//
////         pointCloudRenderer.setPointsToRender(pointsToRender);
//         if (!pointsToRender.isEmpty())
//         {
//            //            pointCloudRenderer.setColor(color);
//            for (int i = 0; i < pointsToRender.size(); i++)
//            {
//               int offset = i * vertexSize;
//
//               Point3D32 point = pointsToRender.get(i);
//               vertices[offset] = point.getX32();
//               vertices[offset + 1] = point.getY32();
//               vertices[offset + 2] = point.getZ32();
//
//               // color [0.0f - 1.0f]
//               vertices[offset + 3] = 0.5f; // red (not working yet)
//               vertices[offset + 4] = 0.7f; // blue
//               vertices[offset + 5] = 0.5f; // green
//               vertices[offset + 6] = alpha; // alpha
//
//               vertices[offset + 7] = 0.11f; // size
//               vertices[offset + 8] = 1.0f; // cosine [0-1]
//               vertices[offset + 9] = 0.0f; // sine [0-1]
//            }
//
//            renderable.meshPart.size = pointsToRender.size();
//            renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.size() * vertexSize);
//            Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);
//            renderable.meshPart.mesh.render(new ShaderProgram("",""),GL20.GL_TRIANGLE_STRIP, 0, 4);
//         }
//      }
//   }
//
//
//   @Override
//   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
//   {
//      if (enabled)
//         renderables.add(renderable);
//   }
//
//   public void setEnabled(boolean enabled)
//   {
//      this.enabled = enabled;
//   }
//}
