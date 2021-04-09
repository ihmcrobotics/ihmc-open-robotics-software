package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import lidar_obstacle_detection.Box3DO32;
import lidar_obstacle_detection.GDXBoxesMessage;
import org.lwjgl.opengl.GL32;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.BoxData;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.ArrayList;

public class GDXROS1BoxVisualizer3 implements RenderableProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_POINTS = 50000;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private ModelInstance modelInstance;
   private Model lastModel;
   private volatile Runnable toRender = null;

   private final ReferenceFrame baseBlockFrame;
   private final FramePoint3D point1 = new FramePoint3D();
   private final FramePoint3D point2 = new FramePoint3D();
   private final FramePoint3D point3 = new FramePoint3D();
   private final FramePoint3D point4 = new FramePoint3D();
   private final FramePoint3D point5 = new FramePoint3D();
   private final FramePoint3D point6 = new FramePoint3D();
   private final FramePoint3D point7 = new FramePoint3D();
   private final FramePoint3D point8 = new FramePoint3D();

   public GDXROS1BoxVisualizer3(RosMainNode ros1Node, String ros1BoxTopic, ReferenceFrame sensorBaseFrame, RigidBodyTransformReadOnly baseToSensorTransform)
   {
      baseBlockFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("baseFrame",
                                                                                           ReferenceFrame.getWorldFrame(),
                                                                                           baseToSensorTransform);

      ros1Node.attachSubscriber(ros1BoxTopic, new AbstractRosTopicSubscriber<GDXBoxesMessage>(GDXBoxesMessage._TYPE)
      {
         @Override
         public void onNewMessage(GDXBoxesMessage boxes)
         {
            queueRenderBoxesAsync(boxes);
         }
      });
   }

   public void render()
   {
      if (toRender != null)
      {
         toRender.run();
         toRender = null;
      }
   }

   private void queueRenderBoxesAsync(GDXBoxesMessage boxes)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(boxes));
   }

   public synchronized void generateMeshes(GDXBoxesMessage boxes)
   {
      BoxData boxData = new BoxData(boxes, MAX_POINTS);
      boxData.flipToZUp();

      Box3DO32[] boxesData = boxData.applyTransform(new RigidBodyTransform());
      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
      for (Box3DO32 box : boxesData)
      {
         point1.setIncludingFrame(baseBlockFrame, box.getPoint1());
         point2.setIncludingFrame(baseBlockFrame, box.getPoint2());
         point3.setIncludingFrame(baseBlockFrame, box.getPoint3());
         point4.setIncludingFrame(baseBlockFrame, box.getPoint4());
         point5.setIncludingFrame(baseBlockFrame, box.getPoint5());
         point6.setIncludingFrame(baseBlockFrame, box.getPoint6());
         point7.setIncludingFrame(baseBlockFrame, box.getPoint7());
         point8.setIncludingFrame(baseBlockFrame, box.getPoint8());

         point1.changeFrame(worldFrame);
         point2.changeFrame(worldFrame);
         point3.changeFrame(worldFrame);
         point4.changeFrame(worldFrame);
         point5.changeFrame(worldFrame);
         point6.changeFrame(worldFrame);
         point7.changeFrame(worldFrame);
         point8.changeFrame(worldFrame);

         double lineWidth = 0.03;
         ArrayList<Point3DReadOnly> orderedVertices = new ArrayList<>();

         orderedVertices.add(point1); // x+y+z+  draw top
         orderedVertices.add(point2); // x-y-z+
         orderedVertices.add(point4); // x-y+z+
         orderedVertices.add(point3); // x+y-z+
         orderedVertices.add(point1); // x+y+z+

         orderedVertices.add(point5); // x+y+z-  go down

         orderedVertices.add(point6); // x-y-z-  leg 1
         orderedVertices.add(point2); // x-y-z+
         orderedVertices.add(point6); // x-y-z-

         orderedVertices.add(point8); // x-y+z-  leg 2
         orderedVertices.add(point4); // x-y+z+
         orderedVertices.add(point8); // x-y+z-

         orderedVertices.add(point7); // x+y-z-  leg 3
         orderedVertices.add(point3); // x+y-z+
         orderedVertices.add(point7); // x+y-z-

         orderedVertices.add(point5); // x+y+z-  leg 4

         meshBuilder.addMultiLine(orderedVertices, lineWidth, Color.RED, false);
      }
      toRender = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(new Color(0.7f, 0.7f, 0.7f, 1.0f)));
         modelBuilder.part(meshPart, material);

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   public void dispose()
   {
      executorService.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // sync over current and add
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }
}
