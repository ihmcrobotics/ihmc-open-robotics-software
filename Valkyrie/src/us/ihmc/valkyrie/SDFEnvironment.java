package us.ihmc.valkyrie;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;
import javax.xml.bind.JAXBException;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFWorldLoader;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.Collision;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.CylinderTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class SDFEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D;

   private final Random random = new Random(1989L);
   
   private static final boolean VISUALIZE_BOUNDING_BOXES = false;
 
   // private static final double FLOOR_THICKNESS = 0.001;
   
   public void load(String filename)
   {
	   try
	      {
	    	FileInputStream f = new FileInputStream(filename);
	    	List<String> resourceDirectories = new ArrayList<String>();
	    	String pth = System.getenv("GAZEBO_RESOURCE_PATH");
	    	if(pth!=null)
	    	{
	    		resourceDirectories= Arrays.asList(System.getenv("GAZEBO_RESOURCE_PATH").split(":"));
	    		for( int i=0;i<resourceDirectories.size();i++)
	    		{
	    			resourceDirectories.set(i, resourceDirectories.get(i)+"/");
	    		}
	    	}
	    	else
	    	{
	    		System.out.println("Environmental variable GAZEBO_RESOURCE_PATH has not been specified!");
	    	}
	    	if(resourceDirectories.size()==0)
	    	{
	    		System.out.println("No reseource paths specified in GAZEBO_RESOURCE_PATH!");
	    	}
	        SDFWorldLoader loader = new SDFWorldLoader(f, resourceDirectories);
	        Graphics3DObject sdf=loader.createGraphics3dObject();
	        ExtractPrimitiveModels(loader);
	        
	        System.out.println("File loaded successfully.");
	      }
	      catch (FileNotFoundException e)
	      {
	    	  System.out.println("SDF loading error:\n"+e.toString());
	    	  e.printStackTrace();
	      }
	      catch (JAXBException e)
	      {
	    	  System.out.println("SDF loading error:\n"+e.toString());
	    	  e.printStackTrace();
	      }   
	      catch (Exception e)
	      {
	    	  System.out.println("SDF loading error:\n"+e.toString());
	    	  e.printStackTrace();
	      } 
   }

   public SDFEnvironment()
   {
      combinedTerrainObject3D = new CombinedTerrainObject3D("SDFEnvironment");
      combinedTerrainObject3D.addTerrainObject(setUpGround("Ground"));      
   }
     
   public void ExtractPrimitiveModels(SDFWorldLoader loader)
   {
	   java.util.Set<String> vis = new java.util.HashSet<String>();
	   vis.addAll(loader.visuals.keySet());
	   if(vis.size()==0)
	   {
		   System.err.println("No models found in the SDF file!");
	   }
	   for ( String key : vis ) 
	   {
           GeneralizedSDFRobotModel model = loader.getGeneralizedRobotModelAndRemoveFromWorld(key);
           ArrayList<SDFLinkHolder> links = model.getRootLinks();
           System.out.println(model.getName());
           for ( SDFLinkHolder link : links )
           {
        	   if(link.getChildren().size()>0)
        	   {
        		   System.out.println("  This is an articulated object - SKIPPING!");
        	   }
        	   else
        	   {
        		   List<Collision> cols = link.getCollisions();
        		   YoAppearanceMaterial mat = new YoAppearanceMaterial();
        		   mat.setAmbientColor(new Color3f(0.5f,0.5f,0.5f));
                   mat.setDiffuseColor(new Color3f(0.5f,0.5f,0.5f));
                   mat.setSpecularColor(new Color3f(0.5f,0.5f,0.5f));
                   
        		   if(link.getVisuals().size()>0)
        		   {
        			   SDFVisual tmpvis = link.getVisuals().get(0);
        			   if(tmpvis.getMaterial()!=null)
        			   {
        				   if(tmpvis.getMaterial().getScript()==null)
        				   {
        					   mat.setAmbientColor(SDFConversionsHelper.stringToColor(tmpvis.getMaterial().getAmbient()));
        					   mat.setDiffuseColor(SDFConversionsHelper.stringToColor(tmpvis.getMaterial().getDiffuse()));
        					   mat.setSpecularColor(SDFConversionsHelper.stringToColor(tmpvis.getMaterial().getSpecular()));
        				   }
        			   }
        		   }
        		   for ( Collision col : cols )
        		   {
        			   RigidBodyTransform transformToModel = new RigidBodyTransform(model.getTransformToRoot());
        		       transformToModel.multiply(link.getTransformFromModelReferenceFrame());
        		       transformToModel.multiply(SDFConversionsHelper.poseToTransform(col.getPose()));
    				   
        			   SDFGeometry geo =col.getGeometry(); 
        			   if (geo.getMesh()!=null) System.out.println("    Mesh geometry is unsupported - SKIPPING!");
        			   if (geo.getImage()!=null) System.out.println("    Image geometry is unsupported - SKIPPING!");
        			   if (geo.getHeightMap()!=null) System.out.println("    Hight map geometry is unsupported - SKIPPING!");
        			   if (geo.getPlane()!=null) System.out.println("    Plane geometry is unsupported - SKIPPING!");
        			   if (geo.getSphere()!=null) System.out.println("    Sphere geometry is unsupported - SKIPPING!");
        			   
        			   if (geo.getBox()!=null)
        			   {
        				   Vector3d sz = SDFConversionsHelper.stringToVector3d(geo.getBox().getSize());
        				   RotatableBoxTerrainObject obj = new RotatableBoxTerrainObject(transformToModel,sz.getX(),sz.getY(),sz.getZ(), mat);
        				   combinedTerrainObject3D.addTerrainObject(obj);
        			   }
        			   if (geo.getCylinder()!=null)
        			   {
        				   double r=Double.parseDouble(geo.getCylinder().getRadius());
        				   double l=Double.parseDouble(geo.getCylinder().getLength());
        				   CylinderTerrainObject obj = new CylinderTerrainObject(transformToModel,l,r,mat);
        				   combinedTerrainObject3D.addTerrainObject(obj);
        			   }
        		   }
        	   }
        	   //processLink(link,"  ");        	           	   
           }
       }
   }

   public static CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3d(0, 0, -0.5));

      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location, 45, 45, 1), YoAppearance.DarkGray());
      combinedTerrainObject.addTerrainObject(newBox2);

      return combinedTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public ArrayList<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }

}

