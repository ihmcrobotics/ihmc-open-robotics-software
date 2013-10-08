package us.ihmc.sensorProcessing.pointClouds.shape;

import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;
import bubo.ptcloud.CloudShapeTypes;
import bubo.ptcloud.FactoryPointCloudShape;
import bubo.ptcloud.PointCloudShapeFinder;
import bubo.ptcloud.alg.CheckShapeCylinderRadius;
import bubo.ptcloud.alg.CheckShapeSphere3DRadius;
import bubo.ptcloud.alg.ConfigSchnabel2007;
import bubo.ptcloud.wrapper.ConfigMergeShapes;
import bubo.ptcloud.wrapper.ConfigSurfaceNormals;
import com.jme3.app.SimpleApplication;
import com.jme3.bounding.BoundingBox;
import com.jme3.input.RawInputListener;
import com.jme3.input.event.JoyAxisEvent;
import com.jme3.input.event.JoyButtonEvent;
import com.jme3.input.event.KeyInputEvent;
import com.jme3.input.event.MouseButtonEvent;
import com.jme3.input.event.MouseMotionEvent;
import com.jme3.input.event.TouchEvent;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.CartoonEdgeFilter;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.shape.Box;

import georegression.struct.point.Point3D_F64;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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
      
      zUp.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());

      rootNode.attachChild(zUp);
      zUp.attachChild(pointCloudNode);
      
      List<Point3D_F64> cloud = readPointCloud(10000000,new Vector3f(-99999.0f,-99999.0f,-99999.0f),new Vector3f(99999.0f,99999.0f,99999.0f));
      displayView(cloud);
      
      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(15);
      setupBoxNode();

//     CloudShapeTypes shapeTypes[] = new CloudShapeTypes[]{CloudShapeTypes.PLANE,CloudShapeTypes.CYLINDER,CloudShapeTypes.SPHERE};
//
//      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(100, 0.6, 0.05, 0.05,shapeTypes);
//     configRansac.randomSeed = 2342342;
//
//      configRansac.minModelAccept = 200;
//      configRansac.octreeSplit = 300;
//zUpNode
//      configRansac.maximumAllowedIterations = 500;
//      configRansac.ransacExtension = 5;
//
//      configRansac.models.get(1).modelCheck = new CheckShapeCylinderRadius(0.2);
//      configRansac.models.get(2).modelCheck = new CheckShapeSphere3DRadius(0.2);
//
//
//      ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(10, 30, Double.MAX_VALUE);
//      ConfigMergeShapes configMerge = new ConfigMergeShapes(0.6, 0.9);
//
//      PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configSurface, configRansac, configMerge);
//
//      shapeFinder.process(cloud, null);
//
//      List<PointCloudShapeFinder.Shape> found = shapeFinder.getFound();
//
//      List<Point3D_F64> unmatched = new ArrayList<Point3D_F64>();
//      shapeFinder.getUnmatched(unmatched);
//
//      System.out.println("Unmatched points " + unmatched.size());
//      System.out.println("total shapes found: " + found.size());
//      int total = 0;
//      for (PointCloudShapeFinder.Shape s : found)
//      {
//         System.out.println("  " + s.type + "  points = " + s.points.size());
//         System.out.println("           " + s.parameters);
//         total += s.points.size();
//      }
//
//      Vector3f[zUpNode] points = new Vector3f[total];
//      ColorRGBA[] colors = new ColorRGBA[total];
//
//      int index = 0;
//      for (PointCloudShapeFinder.Shape s : found)
//      {
//         float r = rand.nextFloat();
//         float g = rand.nextFloat();
//         float b = rand.nextFloat();
//
//         // make sure it is bright enough to see
//         float n = (r + g + b) / 3.0f;
//
//         if (n < 0.5f)
//         {
//            r *= 0.5f / n;
//            g *= 0.5f / n;
//            b *= 0.5f / n;
//         }
//
//
//         ColorRGBA color = new ColorRGBA(r, g, b, 1.0f);
//         System.out.println(" color " + r + " " + g + " " + b);
//
//         for (Point3D_F64 p : s.points)
//         {
//            points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
//            colors[index] = color;
//            index++;
//         }
//      }
//
//
//      PointCloud generator = new PointCloud(assetManager);
//
//      zUp.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
//
//      try
//      {
//         rootNode.attachChild(zUp);
//         zUp.attachChild(generator.generatePointCloudGraph(points, colors));
//      }
//      catch (Exception e)
//      {
//         this.handleError(e.getMessage(), e);
//      }
//
//      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
//      cam.setLocation(new Vector3f(0, 0, -5));
//      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
//      cam.update();
//      flyCam.setMoveSpeed(15);
//
//      setupBoxNode();
//
//      fpp = new FilterPostProcessor(assetManager);
//
//      CartoonEdgeFilter f = new CartoonEdgeFilter();
//      fpp.addFilter(f);
//
//      getViewPort().addProcessor(fpp);
//


   }
   private void ransacRun(List<Point3D_F64> cloud){
	   
     CloudShapeTypes shapeTypes[] = new CloudShapeTypes[]{CloudShapeTypes.PLANE,CloudShapeTypes.CYLINDER,CloudShapeTypes.SPHERE};

      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(100, 0.6, 0.05, 0.05,shapeTypes);
     configRansac.randomSeed = 2342342;

      configRansac.minModelAccept = 200;
      configRansac.octreeSplit = 300;

      configRansac.maximumAllowedIterations = 500;
      configRansac.ransacExtension = 5;

      configRansac.models.get(1).modelCheck = new CheckShapeCylinderRadius(0.2);
      configRansac.models.get(2).modelCheck = new CheckShapeSphere3DRadius(0.2);


      ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(10, 30, Double.MAX_VALUE);
      ConfigMergeShapes configMerge = new ConfigMergeShapes(0.6, 0.9);

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
      for (PointCloudShapeFinder.Shape s : found)
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


         ColorRGBA color = new ColorRGBA(r, g, b, 1.0f);
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
        pointCloudNode.attachChild(generator.generatePointCloudGraph(points, colors));
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

   	   
   }
   private void displayView(List<Point3D_F64> cloud){
	   pointCloudNode.detachAllChildren();
	     System.out.println("total points: " + cloud.size());

	      Vector3f[] points = new Vector3f[cloud.size()];
	      ColorRGBA[] colors = new ColorRGBA[cloud.size()];

	      ColorRGBA color = new ColorRGBA(1, 0, 0, 1.0f);
	      int index = 0;
	      for (Point3D_F64 p : cloud )
	      {
	         points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
	         colors[index] = color;
	         index++;
	      }



	      PointCloud generator = new PointCloud(assetManager);

	      try
	      {
	        
	    	  pointCloudNode.attachChild(generator.generatePointCloudGraph(points, colors));
	         
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
      objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
      objectMaterial.setColor("Color", new ColorRGBA(0, 0, 1, 0.5f));
      geometryBox.setMaterial(objectMaterial);
      geometryBox.setQueueBucket(Bucket.Transparent);

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
        	if(pt.x >= min.x && pt.y>=min.y && pt.z>=min.z && pt.x<=max.x && pt.y<=max.y && pt.z<=max.z) 
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
   private List<Point3D_F64> getBoundedCloud(){
	   Vector3f current =  boundsNode.getLocalTranslation();
       Vector3f max = new Vector3f(current.x+boxExtent,current.y+boxExtent,current.z+boxExtent);
       Vector3f min = new Vector3f(current.x-boxExtent,current.y-boxExtent,current.z-boxExtent);
       return readPointCloud(10000000,min,max);
   }
   @Override
   public void beginInput()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void endInput()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onJoyAxisEvent(JoyAxisEvent evt)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onJoyButtonEvent(JoyButtonEvent evt)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onMouseMotionEvent(MouseMotionEvent evt)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onMouseButtonEvent(MouseButtonEvent evt)
   {
      // TODO Auto-generated method stub

   }

   private boolean ctrlPressed = false;
   private boolean boxHidden = false;
   private FilterPostProcessor fpp;

   @Override
   public void onKeyEvent(KeyInputEvent evt)
   {
      Material objectMaterial = new Material(getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");

//    objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
      objectMaterial.setColor("Color", new ColorRGBA(0, 0, 1, 0.5f));
      boundsNode.setMaterial(objectMaterial);
      boundsNode.setQueueBucket(Bucket.Transparent);



      translateSpeed = 0.05f;

      // up 200 left 203 right 205 down 208
      if ((evt.getKeyCode() == 29) || (evt.getKeyCode() == 157))
      {
         ctrlPressed = evt.isPressed();
      }


      if (ctrlPressed)
      {
         if (evt.getKeyCode() == 23)
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.z += translateSpeed;
            boundsNode.setLocalTranslation(current);
         }
         else if (evt.getKeyCode() == 37)
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.z -= translateSpeed;
            boundsNode.setLocalTranslation(current);
         }
      }
      else 
      {
         if (evt.getKeyCode() == 23)
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.x += translateSpeed;
            boundsNode.setLocalTranslation(current);
         }
         else if (evt.getKeyCode() == 37)
         {
            Vector3f current = boundsNode.getLocalTranslation().clone();
            current.x -= translateSpeed;
            boundsNode.setLocalTranslation(current);
         }
      }

      if (evt.getKeyCode() == 36)
      {
         Vector3f current = boundsNode.getLocalTranslation().clone();
         current.y += translateSpeed;
         boundsNode.setLocalTranslation(current);
      }
      else if (evt.getKeyCode() == 38)
      {
         Vector3f current = boundsNode.getLocalTranslation().clone();
         current.y -= translateSpeed;
         boundsNode.setLocalTranslation(current);
      }

      if (evt.getKeyCode() == 13)
      {
         if (evt.isPressed())
         {
            if (boxHidden)
            {
               boxHidden = false;
               boundsNode.setCullHint(CullHint.Dynamic);
            }
            else
            {
               boxHidden = true;
               boundsNode.setCullHint(CullHint.Always);
            }
         }
      }

      if (evt.getKeyCode() == 28)
      {    	  
          ransacCloud=getBoundedCloud();
          displayView( ransacCloud);
      }

      if (evt.getKeyCode() == 19)
      {
    	  ransacRun(ransacCloud);
      }

   }

   @Override
   public void onTouchEvent(TouchEvent evt)
   {
      // TODO Auto-generated method stub

   }
}
