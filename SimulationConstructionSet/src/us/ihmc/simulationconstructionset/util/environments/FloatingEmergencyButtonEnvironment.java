package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;



public class FloatingEmergencyButtonEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final boolean DEBUG = false;
   private final int NUMBER_OF_CONTACT_POINTS = 50;
	private final List<ContactableButtonRobot> buttonRobots = new ArrayList<ContactableButtonRobot>();
	private final CombinedTerrainObject3D combinedTerrainObject;
	private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
	
	public FloatingEmergencyButtonEnvironment(ArrayList<Point3d> buttonLocations, ArrayList<Vector3d> pushVectors)
	{
		double forceVectorScale = 1.0 / 5.0; 
		
		combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
		combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));
		
		int i = 0;
		for(Point3d buttonLocation : buttonLocations)
		{
		   String buttonRobotName = "ButtonRobot" + i;
		   createButton(buttonRobotName, buttonLocation, pushVectors.get(i), forceVectorScale);
		   if(DEBUG)
	      {
	         System.out.println("Created " + buttonRobotName);
	      }
		   i++;
		}
	}

	private void createButton(String buttonRobotName, Point3d buttonLocation, Vector3d pushVector, double forceVectorScale)
	{
	   
		Matrix3d zRotation = new Matrix3d();
		Matrix3d yRotation = new Matrix3d();
		Matrix3d rotation = new Matrix3d();
		pushVector.normalize();
		
		
		double alpha = Math.atan(pushVector.getY() / pushVector.getX());
		double beta = (Math.PI / 2.0) - Math.atan(pushVector.getZ() / Math.sqrt(Math.pow(pushVector.getX(), 2.0) + Math.pow(pushVector.getY(),2.0)));
		
		zRotation.rotZ(alpha);
		yRotation.rotY(beta);
		rotation.mul(zRotation, yRotation);
		
		RigidBodyTransform rootBodyTransform = new RigidBodyTransform(rotation, new Vector3d(buttonLocation));
	
		ContactableButtonRobot button = new ContactableButtonRobot(buttonRobotName, rootBodyTransform, pushVector);
		
		button.createButtonRobot();
		button.createAvailableContactPoints(1, NUMBER_OF_CONTACT_POINTS, forceVectorScale, true);
		buttonRobots.add(button);
		
	}

	@Override
	public TerrainObject3D getTerrainObject3D()
	{
		return combinedTerrainObject;
	}

	@Override
	public List<ContactableButtonRobot> getEnvironmentRobots()
	{
		return buttonRobots;
	}
	
	@Override
	public void createAndSetContactControllerToARobot() {
	   
	   for(ContactableButtonRobot buttonRobot : this.buttonRobots)
      {
	      ContactController contactController = new ContactController();
	      contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
	      contactController.addContactPoints(contactPoints);
	      contactController.addContactables(buttonRobots);
         buttonRobot.setController(contactController);
      }
	}

	@Override
	public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
	{
		this.contactPoints.addAll(externalForcePoints);

	}

	@Override
	public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
	{
		// TODO Auto-generated method stub

	}
}
