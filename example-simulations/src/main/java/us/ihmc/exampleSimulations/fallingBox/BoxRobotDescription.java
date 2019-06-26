package us.ihmc.exampleSimulations.fallingBox;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class BoxRobotDescription extends RobotDescription
{
   public BoxRobotDescription(String name, Box3D bodyBox, double mass, boolean useGroundContactPoints)
   {
      super(name);

      FloatingJointDescription bodyJoint = new FloatingJointDescription("bodyJoint", "bodyJointVariableName");

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(mass, bodyBox.getSizeX(), bodyBox.getSizeY(), bodyBox.getSizeZ());

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -bodyBox.getSizeZ() / 2.0);
      bodyLinkGraphics.addCube(bodyBox.getSizeX(), bodyBox.getSizeY(), bodyBox.getSizeZ(), YoAppearance.AliceBlue());
      bodyLink.setLinkGraphics(bodyLinkGraphics);
      
      RigidBodyTransform sensorLocation = new RigidBodyTransform();
      ForceSensorDescription ftSensor = new ForceSensorDescription(name + "_ft", sensorLocation);
      ftSensor.setUseGroundContactPoints(useGroundContactPoints);
      bodyJoint.addForceSensor(ftSensor);

      bodyJoint.setLink(bodyLink);

      if (useGroundContactPoints)
      {
         int idOfGCP = 0;
         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, 0.5 * bodyBox.getSizeX(), 0.5 * bodyBox.getSizeY(), -0.5 * bodyBox.getSizeZ());
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, 0.5 * bodyBox.getSizeX(), -0.5 * bodyBox.getSizeY(), -0.5 * bodyBox.getSizeZ());
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, -0.5 * bodyBox.getSizeX(), 0.5 * bodyBox.getSizeY(), -0.5 * bodyBox.getSizeZ());
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, -0.5 * bodyBox.getSizeX(), -0.5 * bodyBox.getSizeY(), -0.5 * bodyBox.getSizeZ());
      }
      
      this.addRootJoint(bodyJoint);
   }
   
   private void addGroundContactPoint(JointDescription joint, String name, double x, double y, double z)
   {
      GroundContactPointDescription gcpDescription = new GroundContactPointDescription(name, new Vector3D(x, y, z));
      joint.addGroundContactPoint(gcpDescription);

      joint.getLink().getLinkGraphics().identity();
      joint.getLink().getLinkGraphics().translate(x, y, z);
      joint.getLink().getLinkGraphics().addSphere(0.01);
   }

}
