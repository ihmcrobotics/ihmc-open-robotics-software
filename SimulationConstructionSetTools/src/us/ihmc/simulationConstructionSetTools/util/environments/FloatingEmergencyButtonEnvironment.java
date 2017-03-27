package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;



public class FloatingEmergencyButtonEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final boolean DEBUG = false;
   private final int NUMBER_OF_CONTACT_POINTS = 50;
	private final List<ContactableButtonRobot> buttonRobots = new ArrayList<ContactableButtonRobot>();
	private final CombinedTerrainObject3D combinedTerrainObject;
	private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
	
	public FloatingEmergencyButtonEnvironment(ArrayList<Point3D> buttonLocations, ArrayList<Vector3D> pushVectors)
	{
		double forceVectorScale = 1.0 / 5.0; 
		
		combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
		combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));
		
		int i = 0;
		for(Point3D buttonLocation : buttonLocations)
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

	private void createButton(String buttonRobotName, Point3D buttonLocation, Vector3D pushVector, double forceVectorScale)
	{
	   
		RotationMatrix zRotation = new RotationMatrix();
		RotationMatrix yRotation = new RotationMatrix();
		RotationMatrix rotation = new RotationMatrix();
		pushVector.normalize();
		
		
		double alpha = Math.atan(pushVector.getY() / pushVector.getX());
		double beta = (Math.PI / 2.0) - Math.atan(pushVector.getZ() / Math.sqrt(Math.pow(pushVector.getX(), 2.0) + Math.pow(pushVector.getY(),2.0)));
		
		zRotation.setToYawMatrix(alpha);
		yRotation.setToPitchMatrix(beta);
		rotation.set(zRotation);
		rotation.multiply(yRotation);
		
		RigidBodyTransform rootBodyTransform = new RigidBodyTransform(rotation, new Vector3D(buttonLocation));
	
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
