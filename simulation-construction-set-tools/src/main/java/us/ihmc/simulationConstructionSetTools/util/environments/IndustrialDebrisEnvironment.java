package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.commons.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class IndustrialDebrisEnvironment implements CommonAvatarEnvironmentInterface
{
   private final ReferenceFrame constructionWorldFrame = ReferenceFrameTools.constructARootFrame("constructionFrame");

   private final CombinedTerrainObject3D combinedTerrainObject;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final List<ContactableSelectableBoxRobot> debrisRobots = new ArrayList<ContactableSelectableBoxRobot>();

   private final String debrisName = "Debris";
   private int id = 0;

   private final double debrisDepth = 0.0508;
   private final double debrisWidth = 0.1016;
   private final double debrisLength = 0.9144;

   private final double debrisMass = 1.0;

   private final PoseReferenceFrame robotInitialPoseReferenceFrame;

   private final double forceVectorScale = 1.0 / 50.0;

   public IndustrialDebrisEnvironment()
   {
      this(new Vector3D(0.0, 0.0, 0.0), 0.0);
   }

   public IndustrialDebrisEnvironment(Tuple3DBasics robotInitialPosition, double robotInitialYaw)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      Quaternion robotInitialOrientation = new Quaternion();
      robotInitialOrientation.setYawPitchRoll(robotInitialYaw, 0.0, 0.0);
      Point3D robotPosition = new Point3D(robotInitialPosition);
      FramePose3D robotInitialPose = new FramePose3D(constructionWorldFrame, robotPosition, robotInitialOrientation);

      this.robotInitialPoseReferenceFrame = new PoseReferenceFrame("robotInitialPoseReferenceFrame", robotInitialPose);
   }

   public void addStandingDebris(double xRelativeToRobot, double yRelativeToRobot, double debrisYaw)
   {
      Point3D tempPosition = new Point3D(xRelativeToRobot, yRelativeToRobot, debrisLength / 2.0);
      FramePose3D debrisPose = generateDebrisPose(tempPosition, debrisYaw, 0.0, 0.0);
      debrisRobots.add(createDebrisRobot(debrisPose));
   }

   public void addHorizontalDebrisLeaningOnTwoBoxes(Point3D positionOfCenterOfDebrisWithRespectToRobot, double debrisYaw, double debrisRoll)
   {
      double supportWidth = 0.1;
      double supportLength = 0.2;
      double supportHeight;

      double x;
      double y;
      double z;

      FramePose3D debrisPose = generateDebrisPose(positionOfCenterOfDebrisWithRespectToRobot, debrisYaw, 0.0, debrisRoll);
      debrisRobots.add(createDebrisRobot(debrisPose));

      RigidBodyTransform debrisTransform = new RigidBodyTransform();
      debrisPose.get(debrisTransform );
      TransformTools.appendRotation(debrisTransform, -debrisRoll, Axis3D.X);
      debrisPose.set(debrisTransform);
      debrisPose.setZ(0.0);
      PoseReferenceFrame debrisReferenceFrame = new PoseReferenceFrame("debrisReferenceFrame", debrisPose);

      //add first support
      FramePose3D firstSupportPose = new FramePose3D(debrisReferenceFrame);
      supportHeight = positionOfCenterOfDebrisWithRespectToRobot.getZ() - debrisWidth / 2.0 + debrisLength / 2.0 * Math.cos(debrisRoll);

      x = 0.0;
      y = -debrisLength / 2.0 * Math.sin(debrisRoll);
      z = supportHeight / 2.0;
      firstSupportPose.getPosition().set(x, y, z);
      firstSupportPose.changeFrame(constructionWorldFrame);
      RigidBodyTransform firstSupportTransform = new RigidBodyTransform();
      firstSupportPose.get(firstSupportTransform);
      combinedTerrainObject.addRotatableBox(firstSupportTransform, supportLength, supportWidth, supportHeight, YoAppearance.AliceBlue());

      //add second support
      FramePose3D secondSupportPose = new FramePose3D(debrisReferenceFrame);
      supportHeight = positionOfCenterOfDebrisWithRespectToRobot.getZ() - debrisWidth / 2.0 - debrisLength / 2.0 * Math.cos(debrisRoll);

      x = 0.0;
      y = debrisLength / 2.0 * Math.sin(debrisRoll);
      z = supportHeight / 2.0;
      secondSupportPose.getPosition().set(x, y, z);
      secondSupportPose.changeFrame(constructionWorldFrame);
      RigidBodyTransform secondSupportTransform = new RigidBodyTransform();
      secondSupportPose.get(secondSupportTransform);
      combinedTerrainObject.addRotatableBox(secondSupportTransform, supportLength, supportWidth, supportHeight, YoAppearance.AliceBlue());
   }

   public void addVerticalDebrisLeaningAgainstAWall(double xRelativeToRobot, double yRelativeToRobot, double debrisYaw, double debrisPitch)
   {
      Point3D tempPosition = new Point3D(xRelativeToRobot, yRelativeToRobot, debrisLength / 2.0 * Math.cos(debrisPitch));
      FramePose3D debrisPose = generateDebrisPose(tempPosition, debrisYaw, debrisPitch, 0.0);
      debrisRobots.add(createDebrisRobot(debrisPose));

      double supportWidth = 0.1;
      double supportLength = 0.2;
      double supportHeight = 1.05*debrisLength;

      RigidBodyTransform debrisTransform = new RigidBodyTransform();
      debrisPose.get(debrisTransform );
      TransformTools.appendRotation(debrisTransform, -debrisPitch, Axis3D.Y);
      debrisPose.set(debrisTransform);
      debrisPose.setZ(0.0);
      PoseReferenceFrame debrisReferenceFrame = new PoseReferenceFrame("debrisReferenceFrame", debrisPose);
      
      FramePose3D supportPose = new FramePose3D(debrisReferenceFrame);
      
      double x = supportWidth / 2.0 + debrisLength/2.0 * Math.sin(debrisPitch);
      double y = 0.0;
      double z = supportHeight / 2.0;
      
      supportPose.getPosition().set(x, y, z);
      supportPose.changeFrame(constructionWorldFrame);
    
      RigidBodyTransform supportTransform = new RigidBodyTransform();
      supportPose.get(supportTransform);
      combinedTerrainObject.addRotatableBox(supportTransform, supportWidth, supportLength, supportHeight, YoAppearance.AliceBlue());
   }

   private FramePose3D generateDebrisPose(Point3D positionWithRespectToRobot, double debrisYaw, double debrisPitch, double debrisRoll)
   {
      FramePose3D debrisPose = new FramePose3D(robotInitialPoseReferenceFrame);
      Quaternion orientation = new Quaternion();
      orientation.setYawPitchRoll(debrisYaw, debrisPitch, debrisRoll);
      debrisPose.set(positionWithRespectToRobot, orientation);
      debrisPose.changeFrame(constructionWorldFrame);
      return debrisPose;
   }

   public void createDebrisContactController()
   {
      for (int i = 0; i < debrisRobots.size(); i++)
      {
         ContactableSelectableBoxRobot debrisRobot = debrisRobots.get(i);
         GroundContactModel groundContactModel = createGroundContactModel(debrisRobot, combinedTerrainObject);
         debrisRobot.createAvailableContactPoints(1, 20, forceVectorScale, true);
         debrisRobot.setGroundContactModel(groundContactModel);
      }
   }

   private ContactableSelectableBoxRobot createDebrisRobot(FramePose3D debrisPose)
   {

      debrisPose.checkReferenceFrameMatch(constructionWorldFrame);
      ContactableSelectableBoxRobot debris = ContactableSelectableBoxRobot.createContactable2By4Robot(debrisName + String.valueOf(id++), debrisDepth,
            debrisWidth, debrisLength, debrisMass);
      debris.setPosition(debrisPose.getX(), debrisPose.getY(), debrisPose.getZ());
      debris.setYawPitchRoll(debrisPose.getYaw(), debrisPose.getPitch(), debrisPose.getRoll());

      return debris;
   }

   private GroundContactModel createGroundContactModel(Robot robot, GroundProfile3D groundProfile)
   {
      double kXY = 5000.0;
      double bXY = 100.0;
      double kZ = 1000.0;
      double bZ = 100.0;
      double alphaStick = 0.7;
      double alphaSlip = 0.5;

      GroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(robot, kXY, bXY, kZ, bZ, alphaSlip, alphaStick,
            robot.getRobotsYoRegistry());
      groundContactModel.setGroundProfile3D(groundProfile);

      return groundContactModel;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<ContactableSelectableBoxRobot> getEnvironmentRobots()
   {
      return debrisRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      // add contact controller to any robot so it gets called
      ContactController contactController = new ContactController("DebrisContactController");
      contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(debrisRobots);
      debrisRobots.get(0).setController(contactController);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }

   public double getDebrisDepth()
   {
      return debrisDepth;
   }

   public double getDebrisWidth()
   {
      return debrisWidth;
   }

   public double getDebrisLength()
   {
      return debrisLength;
   }
}