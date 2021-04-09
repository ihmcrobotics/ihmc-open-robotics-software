package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.shapebuilders.BoxShapeBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import lidar_obstacle_detection.Box3DO32;
import lidar_obstacle_detection.GDXBoxesMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.BoxData;

import us.ihmc.gdx.tools.GDXModelPrimitives;

import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;


import us.ihmc.gdx.ui.graphics.GDXBoxRenderer;

import java.util.HashSet;

public class GDXROS1BoxVisualizer2 implements RenderableProvider
{
   private final float boxSize = 1.0f;
//   private final float distance = 5.0f;

   private RosMainNode ros1Node;
   private String ros1BoxTopic;
   private ReferenceFrame sensorBaseFrame;
   private RigidBodyTransformReadOnly baseToSensorTransform;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private boolean enabled = false;
   private boolean packingA = true;

   private final ModelBuilder modelBuilder = new ModelBuilder();

   private int partIndex = 0;
   private long receivedCount = 0;
   private static final int MAX_POINTS = 50000;

   private Model model;
   private Model lastModel;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private final HashSet<ModelInstance> placedModels = new HashSet<>();

   //   private GDXBoxRenderer boxRenderer = new GDXBoxRenderer();
//   JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

   private RecyclingArrayList<Box3DO32> boxesToRender = new RecyclingArrayList<>(Box3DO32::new);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<Box3DO32> pointsA = new RecyclingArrayList<>(MAX_POINTS, Box3DO32::new);
   private final RecyclingArrayList<Box3DO32> pointsB = new RecyclingArrayList<>(MAX_POINTS, Box3DO32::new);

   private GDXBoxRenderer boxRenderer = new GDXBoxRenderer();
   GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();

   private ModelInstance modelins;

   private ReferenceFrame baseBlockFrame;
   private FramePoint3D point1 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point2 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point3 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point4 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point5 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point6 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point7 = new FramePoint3D(baseBlockFrame, 0, 0, 0);
   private FramePoint3D point8 = new FramePoint3D(baseBlockFrame, 0, 0, 0);

   Color color = new Color(1, 1, 0, 1);


   public GDXROS1BoxVisualizer2(RosMainNode ros1Node, String ros1BoxTopic, ReferenceFrame sensorBaseFrame,
                                RigidBodyTransformReadOnly baseToSensorTransform)
//   public GDXROS1BoxVisualizer2()
   {
      this.ros1Node = ros1Node;
      this.ros1BoxTopic = ros1BoxTopic;
      this.sensorBaseFrame = sensorBaseFrame;
      this.baseToSensorTransform = baseToSensorTransform;

      ros1Node.attachSubscriber(ros1BoxTopic, new AbstractRosTopicSubscriber<GDXBoxesMessage>(GDXBoxesMessage._TYPE)
      {
         @Override
         public void onNewMessage(GDXBoxesMessage boxes)
         {
            ++receivedCount;
            queueRenderBoxes(boxes);
//            System.out.println(boxesToRender.size());
//            meshBuilder.clear();
         }

      });
//      modelBuilder.begin();
//      buildBoxPart(distance, distance, distance, Color.GREEN);
//      buildBoxPart(-distance, distance, distance, Color.DARK_GRAY);
//      buildBoxPart(distance, -distance, distance, Color.RED);
//      buildBoxPart(-distance, -distance, distance, Color.ORANGE);
//      buildBoxPart(distance, distance, -distance, Color.BLUE);
//      buildBoxPart(-distance, distance, -distance, Color.BLACK);
//      buildBoxPart(distance, -distance, -distance, Color.WHITE);
//      buildBoxPart(-distance, -distance, -distance, Color.YELLOW);
//      model = modelBuilder.end();
   }

   private void queueRenderBoxes(GDXBoxesMessage boxes)
   {
      ++receivedCount;
      if (enabled)
      {
         executorService.clearQueueAndExecute(() ->
         {
            try
            {
//               System.out.printf(boxes.getBoundingBoxes()[0].getXMin());

//               boolean hasColors = true;
               BoxData boxData = new BoxData(boxes, MAX_POINTS);

               boxData.flipToZUp();

               // Should be tuned somewhere else
               //               baseToSensorTransform.setToZero();
               //               baseToSensorTransform.appendTranslation(tuneX.get(), tuneY.get(), tuneZ.get());
               //               double pitch = Math.toRadians(90.0 - tunePitch.get());
               //               baseToSensorTransform.appendOrientation(new YawPitchRoll(tuneYaw.get(), pitch, tuneRoll.get()));

//               sensorBaseFrame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
               baseBlockFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("baseFrame", ReferenceFrame.getWorldFrame(), baseToSensorTransform);
//               PoseReferenceFrame stepFrame = new PoseReferenceFrame("stepFrame", ReferenceFrame.getWorldFrame());

//               transformToWorld.multiply(baseToSensorTransform);
//               boxData.applyTransform(transformToWorld);
               Box3DO32[] Boxesdata = boxData.applyTransform(new RigidBodyTransform());

//               Box3DO32[] Boxesdata = boxData.applyTransform(transformToWorld);

               //
               synchronized (boxesToRender)
               {
                  RecyclingArrayList<Box3DO32> boxesToPack = packingA ? pointsA : pointsB;
                  boxesToPack.clear();
//                  System.out.println(BoxData.getNumberOfBoxs());
                  for (int i = 0; i < Boxesdata.length && packingA; i++)
                  {
                     boxesToPack.add().set(Boxesdata[i]);
                  }
                  packingA = !packingA;
               }
            }
            catch (Exception e)
            {
               LogTools.error(e.getMessage());
               e.printStackTrace();
            }
         });
      }
   }

   public void create()
   {
//      boxRenderer.create(MAX_POINTS);
//      meshBuilder.clear();
      placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addBox(0, 0, 0, color), "box"));

   }

   public void update()
   {

//      System.out.println(boxesToRender.size());
      placedModels.clear(); // GUI cleared

      if (enabled)
      {
         boxesToRender.clear();
         synchronized (boxesToRender)
         {
            RecyclingArrayList<Box3DO32> boxesToRead = packingA ? pointsB : pointsA;
            for (Box3DO32 box : boxesToRead)
            {
               boxesToRender.add().set(box);
            }
         }
//         System.out.println(boxesToRender.size());

//         boxRenderer.setBoxRender(boxesToRender);
         if (!boxesToRender.isEmpty()) // boxes update
         {
            modelBuilder.begin();
            if (modelins!=null){
               this.modelins.model.dispose();
//               modelins2.model.dispose();
//               modelins3.model.dispose();
//               modelins4.model.dispose();
//               modelins5.model.dispose();
//               modelins6.model.dispose();
//               modelins7.model.dispose();
//               modelins8.model.dispose();
            }
            for (int i = 0; i < boxesToRender.size(); i++)
            {
               Box3DO32 box3D = boxesToRender.get(i);
//               Point3D point1 = box3D.getPoint1();
//               Point3D point2 = box3D.getPoint2();
//               Point3D point3 = box3D.getPoint3();
//               Point3D point4 = box3D.getPoint4();
//               Point3D point5 = box3D.getPoint5();
//               Point3D point6 = box3D.getPoint6();
//               Point3D point7 = box3D.getPoint7();
//               Point3D point8 = box3D.getPoint8();
//                  FramePoint3D point1 = new FramePoint3D(baseBlockFrame, box3D.getPoint1().getX(), box3D.getPoint1().getY(), box3D.getPoint1().getZ());
//                  FramePoint3D point2 = new FramePoint3D(baseBlockFrame, box3D.getPoint2().getX(), box3D.getPoint2().getY(), box3D.getPoint2().getZ());
//                  FramePoint3D point3 = new FramePoint3D(baseBlockFrame, box3D.getPoint3().getX(), box3D.getPoint3().getY(), box3D.getPoint3().getZ());
//                  FramePoint3D point4 = new FramePoint3D(baseBlockFrame, box3D.getPoint4().getX(), box3D.getPoint4().getY(), box3D.getPoint4().getZ());
//                  FramePoint3D point5 = new FramePoint3D(baseBlockFrame, box3D.getPoint5().getX(), box3D.getPoint5().getY(), box3D.getPoint5().getZ());
//                  FramePoint3D point6 = new FramePoint3D(baseBlockFrame, box3D.getPoint6().getX(), box3D.getPoint6().getY(), box3D.getPoint6().getZ());
//                  FramePoint3D point7 = new FramePoint3D(baseBlockFrame, box3D.getPoint7().getX(), box3D.getPoint7().getY(), box3D.getPoint7().getZ());
//                  FramePoint3D point8 = new FramePoint3D(baseBlockFrame, box3D.getPoint8().getX(), box3D.getPoint8().getY(), box3D.getPoint8().getZ());
               this.point1.setReferenceFrame(baseBlockFrame);
               this.point1.setX(box3D.getPoint1().getX());
               this.point1.setY(box3D.getPoint1().getY());
               this.point1.setZ(box3D.getPoint1().getZ());

               this.point2.setReferenceFrame(baseBlockFrame);
               this.point2.setX(box3D.getPoint2().getX());
               this.point2.setY(box3D.getPoint2().getY());
               this.point2.setZ(box3D.getPoint2().getZ());

               this.point3.setReferenceFrame(baseBlockFrame);
               this.point3.setX(box3D.getPoint3().getX());
               this.point3.setY(box3D.getPoint3().getY());
               this.point3.setZ(box3D.getPoint3().getZ());

               this.point4.setReferenceFrame(baseBlockFrame);
               this.point4.setX(box3D.getPoint4().getX());
               this.point4.setY(box3D.getPoint4().getY());
               this.point4.setZ(box3D.getPoint4().getZ());

               this.point5.setReferenceFrame(baseBlockFrame);
               this.point5.setX(box3D.getPoint5().getX());
               this.point5.setY(box3D.getPoint5().getY());
               this.point5.setZ(box3D.getPoint5().getZ());

               this.point6.setReferenceFrame(baseBlockFrame);
               this.point6.setX(box3D.getPoint6().getX());
               this.point6.setY(box3D.getPoint6().getY());
               this.point6.setZ(box3D.getPoint6().getZ());

               this.point7.setReferenceFrame(baseBlockFrame);
               this.point7.setX(box3D.getPoint7().getX());
               this.point7.setY(box3D.getPoint7().getY());
               this.point7.setZ(box3D.getPoint7().getZ());

               this.point8.setReferenceFrame(baseBlockFrame);
               this.point8.setX(box3D.getPoint8().getX());
               this.point8.setY(box3D.getPoint8().getY());
               this.point8.setZ(box3D.getPoint8().getZ());

//               this.point1.set(box3D.getPoint1().getX(), box3D.getPoint1().getY(), box3D.getPoint1().getZ());
//               this.point2.set(box3D.getPoint2().getX(), box3D.getPoint2().getY(), box3D.getPoint2().getZ());
//               this.point3.set(box3D.getPoint3().getX(), box3D.getPoint3().getY(), box3D.getPoint3().getZ());
//               this.point4.set(box3D.getPoint4().getX(), box3D.getPoint4().getY(), box3D.getPoint4().getZ());
//               this.point5.set(box3D.getPoint5().getX(), box3D.getPoint5().getY(), box3D.getPoint5().getZ());
//               this.point6.set(box3D.getPoint6().getX(), box3D.getPoint6().getY(), box3D.getPoint6().getZ());
//               this.point7.set(box3D.getPoint7().getX(), box3D.getPoint7().getY(), box3D.getPoint7().getZ());
//               this.point8.set(box3D.getPoint8().getX(), box3D.getPoint8().getY(), box3D.getPoint8().getZ());

               this.point1.changeFrame(worldFrame);
               this.point2.changeFrame(worldFrame);
               this.point3.changeFrame(worldFrame);
               this.point4.changeFrame(worldFrame);
               this.point5.changeFrame(worldFrame);
               this.point6.changeFrame(worldFrame);
               this.point7.changeFrame(worldFrame);
               this.point8.changeFrame(worldFrame);

               buildBoxPart((float)((float)(this.point1.getY()+ this.point8.getY())/2), (float)((this.point1.getX()+ this.point8.getX())/2),
                            (float)((this.point1.getZ()+ this.point8.getZ())/2), (float)(this.point1.getY()- this.point8.getY()),
                            (float)(this.point1.getX()- this.point8.getX()), (float)(this.point1.getZ()- this.point8.getZ()),Color.RED);



//               this.modelins = GDXModelPrimitives.buildModelInstance(meshBuilder->meshBuilder.addLine(point1.getY(), point1.getX(), point1.getZ(),
//                                             point3.getY(),point3.getX(),point3.getZ(), 0.1, color),"box_line");
//               placedModels.add(this.modelins);

//               modelins2 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point1.getY(), point1.getX(), point1.getZ(),
//                                             point5.getY(),point5.getX(),point5.getZ(), 0.1, color), "box_line_2");
//               placedModels.add(modelins2);
////
//               modelins3 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point3.getY(), point3.getX(), point3.getZ(),
//                                             point7.getY(),point7.getX(),point7.getZ(), 0.1, color), "box_line");
//               placedModels.add(modelins3);
//
//               modelins4 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point5.getY(), point5.getX(), point5.getZ(),
//                                             point7.getY(),point7.getX(),point7.getZ(), 0.1, color), "box_line");
//               placedModels.add(modelins4);
//////////////////////////////////////////////////////////////////////
//               modelins5 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point2.getY(), point2.getX(), point2.getZ(),
//                                             point4.getY(),point4.getX(),point4.getZ(), 0.1, color), "box_line");
//               placedModels.add(modelins5);
//
//               modelins6 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point2.getY(), point2.getX(), point2.getZ(),
//                                             point6.getY(),point6.getX(),point6.getZ(), 0.1, color), "box_line");
//               placedModels.add(modelins6);
//
//               modelins7 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point4.getY(), point4.getX(), point4.getZ(),
//                                             point8.getY(),point8.getX(),point8.getZ(), 0.1, color), "box_line");
//               placedModels.add(modelins7);
//
//               modelins8 = GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point6.getY(), point6.getX(), point6.getZ(),
//                                             point8.getY(),point8.getX(),point8.getZ(), 0.1, color), "box_line");
//               placedModels.add(modelins8);
//
//               placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point1.getY(), point1.getX(), point1.getZ(),
//                                             point2.getY(),point2.getX(),point2.getZ(), 0.1, color), "box_line"));
//               placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point3.getY(), point3.getX(), point3.getZ(),
//                                             point4.getY(),point4.getX(),point4.getZ(), 0.1, color), "box_line"));
//               placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point5.getY(), point5.getX(), point5.getZ(),
//                                             point6.getY(),point6.getX(),point6.getZ(), 0.1, color), "box_line"));
//               placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addLine(point7.getY(), point7.getX(), point7.getZ(),
//                                             point8.getY(),point8.getX(),point8.getZ(), 0.1, color), "box_line"));


               //               float lx = (float) (box3D.getXMax() - box3D.getXMin());
//               float ly = (float) (box3D.getYMax() - box3D.getYMin());
//               float lz = (float) (box3D.getZMax() - box3D.getZMin());
//               double center_x = (box3D.getXMax() + box3D.getXMin())/2;
//               double center_y = (box3D.getYMax() + box3D.getYMin())/2;
//               double center_z = (box3D.getZMax() + box3D.getZMin())/2;
//
//               Tuple3DReadOnly center_point = new Point3D((float)center_x, (float)center_y, (float)center_z);
//               placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addBox(1, 1, 1,
//                                               box3D.getTransform().getTranslation(), new Color(1, 1, 0, 1)), "box"));

               //               placedModels.add(GDXModelPrimitives.buildModelInstance(meshBuilder -> meshBuilder.addBox(0, 0, 0, new Color(1, 1, 1, 1)), "box"));


            }
            try
            {
               lastModel.dispose();
            }
            catch(Exception e){
            }
            lastModel = modelBuilder.end();
            modelins = new ModelInstance(lastModel);
            placedModels.add(modelins);
         }

      }
//      boundingBoxGraphics.removeAll();
//      System.out.println(boxesToRender.size());
//      javafx.scene.paint.Color[] boundingBoxColors = new javafx.scene.paint.Color[] {javafx.scene.paint.Color.INDIANRED, javafx.scene.paint.Color.DARKSEAGREEN, javafx.scene.paint.Color.CADETBLUE};
//
//      for (int i = 0; i < boxesToRender.size(); i++)
//      {
//         javafx.scene.paint.Color color = boundingBoxColors[i % boundingBoxColors.length];
//         boundingBox = new BoundingBox3D();
//         boundingBox.setMin(boxesToRender.get(i).getXMin(), boxesToRender.get(i).getYMin(), boxesToRender.get(i).getZMin());
//         boundingBox.setMax(boxesToRender.get(i).getXMax(), boxesToRender.get(i).getYMax(), boxesToRender.get(i).getZMax());
//
//         boundingBoxGraphics.add(createBoundingBox3D(boundingBox, color, 0.02));
//      }

   }

   public void setEnabled(boolean flag)
   {
      this.enabled = flag;
//      boundingBoxGraphics.setEnabled(enabled);
   }

   private void buildBoxPart(float x, float y, float z, float boxSize_x, float boxSize_y, float boxSize_z, Color color)
   {
      Node node = modelBuilder.node();
      node.translation.set(x, y, z);
      MeshPartBuilder partBuilder = modelBuilder.part("box" + partIndex++,
                                                      GL20.GL_TRIANGLES,
                                                      VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal,
                                                      new Material(ColorAttribute.createDiffuse(color)));
      BoxShapeBuilder.build(partBuilder, boxSize_x, boxSize_y, boxSize_z);
   }


//   public ModelInstance newInstance()
//   {
//      return new ModelInstance(model);
//   }

   public ModelInstance Instance()
   {
      return new ModelInstance(model);
   }


   public void dispose()
   {
//      model.dispose();
      executorService.destroy();
   }

//   @Override
//   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
//   {
//      if (enabled)
//         boxRenderer.getRenderables(renderables, pool);
//   }
   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance placedModel : placedModels)
      {
         placedModel.getRenderables(renderables, pool);
      }
   }

}
