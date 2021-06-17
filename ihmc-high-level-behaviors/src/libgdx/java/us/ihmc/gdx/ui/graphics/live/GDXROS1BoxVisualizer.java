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
import lidar_obstacle_detection.GDXBoxMessage;
import lidar_obstacle_detection.GDXBoxesMessage;
import org.lwjgl.opengl.GL32;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1BoxVisualizer extends ImGuiGDXROS1Visualizer implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private ModelInstance modelInstance;
   private Model lastModel;
   private volatile Runnable toRender = null;

   private AbstractRosTopicSubscriber<GDXBoxesMessage> subscriber;
   private final String ros1BoxTopic;
   private ReferenceFrame sensorFrame;
   private final FrameBoundingBox3D boundingBox = new FrameBoundingBox3D();
   private final FrameBox3D box = new FrameBox3D();
   private final Point3D center = new Point3D();
   private final Quaternion zeroOrientation = new Quaternion();
   private final Point3D[] vertices = new Point3D[8];
   private Color color = new Color(0.7f, 0.7f, 0.7f, 1.0f);

   public GDXROS1BoxVisualizer(String title, String ros1BoxTopic)
   {
      super(title);
      this.ros1BoxTopic = ros1BoxTopic;

      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      sensorFrame = ReferenceFrame.getWorldFrame();
   }

   public void setFrame(ReferenceFrame sensorBaseFrame, RigidBodyTransformReadOnly baseToSensorTransform)
   {
      sensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("baseFrame",
                                                                                        sensorBaseFrame,
                                                                                        baseToSensorTransform);
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      subscriber = new AbstractRosTopicSubscriber<GDXBoxesMessage>(GDXBoxesMessage._TYPE)
      {
         @Override
         public void onNewMessage(GDXBoxesMessage boxes)
         {
            queueRenderBoxesAsync(boxes);
         }
      };
      ros1Node.attachSubscriber(ros1BoxTopic, subscriber);
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
   }

   @Override
   public void update()
   {
      super.update();
      if (toRender != null)
      {
         toRender.run();
         toRender = null;
      }
   }

   public void setColor(Color color)
   {
      this.color.set(color);
   }

   private void queueRenderBoxesAsync(GDXBoxesMessage boxes)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(boxes));
   }

   public synchronized void generateMeshes(GDXBoxesMessage boxes)
   {
      double lineWidth = 0.03;
      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
      for (GDXBoxMessage message : boxes.getBoxes())
      {
         boundingBox.setToZero(sensorFrame);

         // Be robust to incorrect incoming data
//         double xMin = Math.min(message.getXMin(), message.getXMax());
//         double yMin = Math.min(message.getYMin(), message.getYMax());
//         double zMin = Math.min(message.getZMin(), message.getZMax());
//         double xMax = Math.max(message.getXMin(), message.getXMax());
//         double yMax = Math.max(message.getYMin(), message.getYMax());
//         double zMax = Math.max(message.getZMin(), message.getZMax());

         double xMin = message.getXMin();
         double yMin = message.getYMin();
         double zMin = message.getZMin();
         double xMax = message.getXMax();
         double yMax = message.getYMax();
         double zMax = message.getZMax();

         assert(xMin<xMax);
         assert(yMin<yMax);
         assert(zMin<zMax);

//         boundingBox.set(-yMax, -xMax, zMin, -yMin, -xMin, zMax); // x and y Flip to up + xy plane mirrors
         boundingBox.set(zMin,-xMax, -yMax, zMax, -xMin, -yMin); // from X: right Z: inward to x:inward Z: upward
         boundingBox.getCenterPoint(center);
         box.setIncludingFrame(sensorFrame,
                               center,
                               zeroOrientation,
                               boundingBox.getMaxX() - boundingBox.getMinX(),
                               boundingBox.getMaxY() - boundingBox.getMinY(),
                               boundingBox.getMaxZ() - boundingBox.getMinZ());
         box.changeFrame(ReferenceFrame.getWorldFrame());
         box.getVertices(vertices);

         meshBuilder.addMultiLineBox(vertices, lineWidth, Color.RED);
      }

      toRender = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         material.set(ColorAttribute.createDiffuse(color));
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
