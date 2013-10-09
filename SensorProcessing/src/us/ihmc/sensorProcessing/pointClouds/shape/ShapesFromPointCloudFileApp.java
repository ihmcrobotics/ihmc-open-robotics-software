package us.ihmc.sensorProcessing.pointClouds.shape;

import bubo.ptcloud.wrapper.ConfigRemoveFalseShapes;
import georegression.struct.point.Point3D_F64;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Callable;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;
import bubo.ptcloud.CloudShapeTypes;
import bubo.ptcloud.FactoryPointCloudShape;
import bubo.ptcloud.PointCloudShapeFinder;
import bubo.ptcloud.alg.ConfigSchnabel2007;
import bubo.ptcloud.shape.CheckShapeCylinderRadius;
import bubo.ptcloud.shape.CheckShapeSphere3DRadius;
import bubo.ptcloud.wrapper.ConfigMergeShapes;
import bubo.ptcloud.wrapper.ConfigSurfaceNormals;

import com.jme3.app.SimpleApplication;
import com.jme3.input.RawInputListener;
import com.jme3.input.event.JoyAxisEvent;
import com.jme3.input.event.JoyButtonEvent;
import com.jme3.input.event.KeyInputEvent;
import com.jme3.input.event.MouseButtonEvent;
import com.jme3.input.event.MouseMotionEvent;
import com.jme3.input.event.TouchEvent;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.CartoonEdgeFilter;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.shape.Box;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;

/**
 * @author Peter Abeles
 */
public class ShapesFromPointCloudFileApp extends SimpleApplication implements RawInputListener
{
   private Random rand = new Random(234);

   public String fileName;
   private List<Point3D_F64> ransacCloud;
   private Node boundsNode = new Node("meshBounds");
   private float boxExtent = 0.5f;
   private Node zUp = new Node();
   private Node pointCloudNode = new Node();
   private float translateSpeed = 0.1f;
   private ShapeTranslator translator = new ShapeTranslator(this);
   private Node shapesNode = new Node();
   private DirectionalLight primaryLight;
   private List<Point3D_F64> fullCloud;
   private ColorRGBA defaultColor = ColorRGBA.Red;
   private ColorRGBA selectColor = ColorRGBA.Blue;

   private float defaultPointSize = 0.75f;
   private float selectPointSize = 2.0f;

   private Node selectionNode = new Node();


   public static void main(String[] args)
   {
      ShapesFromPointCloudFileApp test1 = new ShapesFromPointCloudFileApp("../SensorProcessing/output.txt");
      test1.start();
   }

   public ShapesFromPointCloudFileApp(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      inputManager.addRawInputListener(this);
      setupLighting();
      zUp.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());

      rootNode.attachChild(zUp);
      zUp.attachChild(pointCloudNode);
      zUp.attachChild(shapesNode);
      zUp.attachChild(selectionNode);


      fullCloud = readPointCloud(10000000, new Vector3f(-99999.0f, -99999.0f, -99999.0f), new Vector3f(99999.0f, 99999.0f, 99999.0f));
      displayView(fullCloud, defaultColor, defaultPointSize, pointCloudNode);

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(15);
      flyCam.setDragToRotate(true);


      FilterPostProcessor fpp = new FilterPostProcessor(assetManager);

      CartoonEdgeFilter f = new CartoonEdgeFilter();
      fpp.addFilter(f);

      getViewPort().addProcessor(fpp);
      setupBoxNode();


   }

   private void setupLighting()
   {
      primaryLight = setupDirectionalLight(new Vector3f(-0.5f, -8, -2));
      rootNode.addLight(primaryLight);

      DirectionalLight d2 = setupDirectionalLight(new Vector3f(1, -1, 1));
      rootNode.addLight(d2);

      DirectionalLight d3 = setupDirectionalLight(new Vector3f(1, -1, -1));
      rootNode.addLight(d3);

      DirectionalLight d4 = setupDirectionalLight(new Vector3f(-1, -1, 1));
      rootNode.addLight(d4);

      AmbientLight a1 = new AmbientLight();
      a1.setColor(ColorRGBA.White.mult(0.4f));
      rootNode.addLight(a1);

      DirectionalLightShadowRenderer dlsr = new DirectionalLightShadowRenderer(assetManager, 2048, 2);
      dlsr.setLight(primaryLight);
      dlsr.setLambda(0.3f);
      dlsr.setShadowIntensity(0.4f);
      dlsr.setEdgeFilteringMode(EdgeFilteringMode.Dither);

      // dlsr.displayFrustum();
      viewPort.addProcessor(dlsr);

      rootNode.setShadowMode(ShadowMode.Off);
      zUp.setShadowMode(ShadowMode.Off);

   }

   private DirectionalLight setupDirectionalLight(Vector3f direction)
   {
      DirectionalLight d2 = new DirectionalLight();
      d2.setColor(ColorRGBA.White.mult(0.6f));
      Vector3f lightDirection2 = direction.normalizeLocal();
      d2.setDirection(lightDirection2);

      return d2;
   }

   private void ransacRun(final List<Point3D_F64> cloud)
   {
      System.err.println("RUNNING RANSAC PLEASE HOLD");
      enqueue(new Callable<Object>()
      {
         public Object call() throws Exception
         {
            shapesNode.detachAllChildren();

            return null;
         }
      });
      Thread ransac = new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            // TODO Auto-generated method stub

            CloudShapeTypes shapeTypes[] = new CloudShapeTypes[] {CloudShapeTypes.PLANE, CloudShapeTypes.CYLINDER, CloudShapeTypes.SPHERE};

            ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(20, 0.8, 0.03, shapeTypes);
            configRansac.randomSeed = 2342342;

            configRansac.minModelAccept = 100;
            configRansac.octreeSplit = 25;

            configRansac.maximumAllowedIterations = 1000;
            configRansac.ransacExtension = 25;

            configRansac.models.get(1).modelCheck = new CheckShapeCylinderRadius(0.2);
            configRansac.models.get(2).modelCheck = new CheckShapeSphere3DRadius(0.2);


            ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(10, 30, Double.MAX_VALUE);
            ConfigRemoveFalseShapes configMerge = new ConfigRemoveFalseShapes(0.7);

            PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configSurface, configRansac, configMerge);

            shapeFinder.process(cloud, null);

            List<PointCloudShapeFinder.Shape> found = shapeFinder.getFound();

            List<Point3D_F64> unmatched = new ArrayList<Point3D_F64>();
            shapeFinder.getUnmatched(unmatched);

            System.out.println("Unmatched points " + unmatched.size());
            System.out.println("total shapes found: " + found.size());
            int total = 0;
            for (PointCloudShapeFinder.Shape s : found)
            {
               System.out.println("  " + s.type + "  points = " + s.points.size());
               System.out.println("           " + s.parameters);
               total += s.points.size();
            }

            pointCloudNode.detachAllChildren();
            Vector3f[] points = new Vector3f[total];
            ColorRGBA[] colors = new ColorRGBA[total];

            int index = 0;
            for (final PointCloudShapeFinder.Shape s : found)
            {
               float r = rand.nextFloat();
               float g = rand.nextFloat();
               float b = rand.nextFloat();

               // make sure it is bright enough to see
               float n = (r + g + b) / 3.0f;

               if (n < 0.5f)
               {
                  r *= 0.5f / n;
                  g *= 0.5f / n;
                  b *= 0.5f / n;
               }


               final ColorRGBA color = new ColorRGBA(r, g, b, 1.0f);

               enqueue(new Callable<Object>()
               {
                  public Object call() throws Exception
                  {
                     translator.translateShape(s, color, shapesNode);

                     return null;
                  }
               });


               System.out.println(" color " + r + " " + g + " " + b);

               for (Point3D_F64 p : s.points)
               {
                  points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
                  colors[index] = color;
                  index++;
               }
            }


            PointCloud generator = new PointCloud(assetManager);

            try
            {
               final Node pointCloudNodeToAdd = generator.generatePointCloudGraph(points, colors, defaultPointSize);
               enqueue(new Callable<Object>()
               {
                  public Object call() throws Exception
                  {
                     selectionNode.detachAllChildren();
                     pointCloudNode.attachChild(pointCloudNodeToAdd);

                     return null;
                  }
               });
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }

            System.out.println("RANSAC COMPLETE THANK YOU FOR YOUR PATIENCE");

         }
      });
      ransac.start();


   }

   private void clearView()
   {
      pointCloudNode.detachAllChildren();

   }

   private void displayView(List<Point3D_F64> cloud, ColorRGBA color, float size, Node nodeToAddTo)
   {
      System.out.println("total points: " + cloud.size());

      Vector3f[] points = new Vector3f[cloud.size()];
      ColorRGBA[] colors = new ColorRGBA[cloud.size()];


      int index = 0;
      for (Point3D_F64 p : cloud)
      {
         points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
         colors[index] = color;
         index++;
      }



      PointCloud generator = new PointCloud(assetManager);

      try
      {
         nodeToAddTo.attachChild(generator.generatePointCloudGraph(points, colors, size));

      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

   }

   private void setupBoxNode()
   {
      Box box = new Box(boxExtent, boxExtent, boxExtent);
      Geometry geometryBox = new Geometry("boxMesh", box);

      Material objectMaterial = new Material(getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      objectMaterial.setColor("Color", new ColorRGBA(0, 0, 1, 0.5f));
      geometryBox.setMaterial(objectMaterial);
      geometryBox.setQueueBucket(Bucket.Transparent);
      geometryBox.setShadowMode(ShadowMode.CastAndReceive);


      boundsNode.attachChild(geometryBox);
      zUp.attachChild(boundsNode);

   }

   private List<Point3D_F64> readPointCloud(int maxLines, Vector3f min, Vector3f max)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      SerializationDefinitionManager manager = new SerializationDefinitionManager();
      manager.addDefinition(new DataDefinition("point3d", Point3D_F64.class, "x", "y", "z"));

      try
      {
         FileInputStream in = new FileInputStream(fileName);
         ReadCsvObjectSmart<Point3D_F64> reader = new ReadCsvObjectSmart<Point3D_F64>(in, manager, "point3d");


         Point3D_F64 pt = new Point3D_F64();
         int count = 0;
         while ((reader.nextObject(pt) != null) && (count++ < maxLines))
         {
            if ((pt.x >= min.x) && (pt.y >= min.y) && (pt.z >= min.z) && (pt.x <= max.x) && (pt.y <= max.y) && (pt.z <= max.z))
               cloud.add(pt.copy());
         }

      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      return cloud;
   }

   private List<Point3D_F64> getBoundedCloud()
   {
      Vector3f current = boundsNode.getLocalTranslation();
      Vector3f max = new Vector3f(current.x + boxExtent, current.y + boxExtent, current.z + boxExtent);
      Vector3f min = new Vector3f(current.x - boxExtent, current.y - boxExtent, current.z - boxExtent);

      return readPointCloud(10000000, min, max);
   }


   public void beginInput()
   {
      // TODO Auto-generated method stub

   }


   public void endInput()
   {
      // TODO Auto-generated method stub

   }


   public void onJoyAxisEvent(JoyAxisEvent evt)
   {
      // TODO Auto-generated method stub

   }


   public void onJoyButtonEvent(JoyButtonEvent evt)
   {
      // TODO Auto-generated method stub

   }


   public void onMouseMotionEvent(MouseMotionEvent evt)
   {
      // TODO Auto-generated method stub

   }


   public void onMouseButtonEvent(MouseButtonEvent evt)
   {
      // TODO Auto-generated method stub

   }

   private boolean ctrlPressed = false;
   private boolean boxHidden = false;
   private FilterPostProcessor fpp;

   public void onKeyEvent(KeyInputEvent evt)
   {
      Material objectMaterial = new Material(getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      objectMaterial.setColor("Color", new ColorRGBA(0, 0, 1, 0.5f));
      boundsNode.setMaterial(objectMaterial);
      boundsNode.setQueueBucket(Bucket.Transparent);
      boundsNode.setShadowMode(ShadowMode.CastAndReceive);




      translateSpeed = 0.05f;

      // up 200 left 203 right 205 down 208
      if ((evt.getKeyCode() == 29) || (evt.getKeyCode() == 157))
      {
         ctrlPressed = evt.isPressed();
      }


      if (ctrlPressed)
      {
         if ((evt.getKeyCode() == 23) && evt.isPressed())
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.z += translateSpeed;
            boundsNode.setLocalTranslation(current);
            updateAfterMoveIfRequired();
         }
         else if ((evt.getKeyCode() == 37) && evt.isPressed())
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.z -= translateSpeed;
            boundsNode.setLocalTranslation(current);
            updateAfterMoveIfRequired();
         }
      }
      else
      {
         if ((evt.getKeyCode() == 23) && evt.isPressed())
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.x += translateSpeed;
            boundsNode.setLocalTranslation(current);
            updateAfterMoveIfRequired();
         }
         else if ((evt.getKeyCode() == 37) && evt.isPressed())
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.x -= translateSpeed;
            boundsNode.setLocalTranslation(current);
            updateAfterMoveIfRequired();
         }
      }

      if ((evt.getKeyCode() == 36) && evt.isPressed())
      {
         Vector3f current = boundsNode.getLocalTranslation().clone();
         current.y += translateSpeed;
         boundsNode.setLocalTranslation(current);
         updateAfterMoveIfRequired();
      }
      else if ((evt.getKeyCode() == 38) && evt.isPressed())
      {
         Vector3f current = boundsNode.getLocalTranslation().clone();
         current.y -= translateSpeed;
         boundsNode.setLocalTranslation(current);
         updateAfterMoveIfRequired();
      }

      if (evt.getKeyCode() == 13)
      {
         if (evt.isPressed())
         {
            if (boxHidden)
            {
               boxHidden = false;
               shapesNode.setCullHint(CullHint.Dynamic);
            }
            else
            {
               boxHidden = true;
               shapesNode.setCullHint(CullHint.Always);
            }
         }
      }

      if ((evt.getKeyCode() == 28) && evt.isPressed())
      {
         // hide the box
         boundsNode.setCullHint(CullHint.Always);
         clearView();
         showBounds();

      }

      if ((evt.getKeyCode() == 19) && evt.isPressed())
      {
         ransacRun(ransacCloud);
      }

   }

   private void showBounds()
   {
      calculatingBounds = true;
      Thread boundsThread = new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            ransacCloud = getBoundedCloud();
            enqueue(new Callable<Object>()
            {
               public Object call() throws Exception
               {
                  selectionNode.detachAllChildren();
                  displayView(ransacCloud, selectColor, selectPointSize, selectionNode);
                  calculatingBounds = false;

                  return null;
               }
            });

         }
      });
      boundsThread.start();
   }

   private boolean calculatingBounds = false;

   private void updateAfterMoveIfRequired()
   {
      if (!calculatingBounds)
      {
       showBounds();
      }


      if (boundsNode.getCullHint().equals(CullHint.Always))
      {
         enqueue(new Callable<Object>()
         {
            public Object call() throws Exception
            {
               shapesNode.detachAllChildren();
               displayView(fullCloud, defaultColor, defaultPointSize, pointCloudNode);

               boundsNode.setCullHint(CullHint.Dynamic);

               return null;
            }
         });

      }
   }

   public void onTouchEvent(TouchEvent evt)
   {
      // TODO Auto-generated method stub

   }
}
