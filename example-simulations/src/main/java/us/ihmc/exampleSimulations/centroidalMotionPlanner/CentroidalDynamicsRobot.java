package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;
import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CentroidalDynamicsRobot implements FullRobotModelFactory
{
   private final double robotMass;
   private final double Ixx;
   private final double Iyy;
   private final double Izz;
   private final double xRadius;
   private final double yRadius;
   private final double zRadius;
   private final DenseMatrix64F momentOfInertia;
   private final double robotHeight = 0.75;
   private final String robotName;

   private final RobotDescription robotDescription;
   private final JointNameMap<RobotCentroidal> jointMap;

   public CentroidalDynamicsRobot(String robotName)
   {
      this.robotName = robotName;
      this.robotMass = 18.0;
      this.Ixx = 0.1;
      this.Iyy = 0.1;
      this.Izz = 0.1;
      this.xRadius = 0.1;
      this.yRadius = 0.1;
      this.zRadius = 0.1;
      this.momentOfInertia = new DenseMatrix64F(3, 3);
      momentOfInertia.set(0, 0, Ixx);
      momentOfInertia.set(1, 1, Iyy);
      momentOfInertia.set(2, 2, Izz);
      this.robotDescription = new CentroidalRobotDescription(robotName);
      this.jointMap = new CentroidalRobotJointMap();
   }

   public Robot getSCSRobot()
   {
      RobotFromDescription scsRobot = new RobotFromDescription(getRobotDescription());
      scsRobot.setController(new CentroidaDynamicsRobotController());
      CentroidalDynamicsRobotInitialSetup initialSetup = new CentroidalDynamicsRobotInitialSetup();
      initialSetup.initializeRobot(scsRobot);
      return scsRobot;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public FullRobotModel createFullRobotModel()
   {
      RobotDescription robotDescription = new CentroidalRobotDescription(robotName);
      FullRobotModelFromDescription fullRobotModel = new FullRobotModelFromDescription(robotDescription, getJointMap(), null);
      return fullRobotModel;
   }

   public JointNameMap<RobotCentroidal> getJointMap()
   {
      return jointMap;
   }

   public enum RobotCentroidal implements RobotSegment<RobotCentroidal>
   {
      BODY;

      public static final RobotCentroidal[] values = values();
      public static final EnumSet<RobotCentroidal> set = EnumSet.allOf(RobotCentroidal.class);

      @Override
      public RobotCentroidal[] getValues()
      {
         return values;
      }

      @Override
      public Class<RobotCentroidal> getClassType()
      {
         return RobotCentroidal.class;
      }

      @Override
      public EnumSet<RobotCentroidal> getEnumSet()
      {
         return set;
      }
   }

   public class CentroidalDynamicsRobotInitialSetup
   {
      Vector3D initialPosition = new Vector3D();
      Quaternion orientation = new Quaternion();

      public CentroidalDynamicsRobotInitialSetup()
      {
         initialPosition.set(0.0, 0.0, robotHeight);
      }

      public void initializeRobot(Robot robot)
      {
         FloatingJoint rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
         rootJoint.setPosition(initialPosition);
         rootJoint.setQuaternion(orientation);
      }

      public void setInitialYaw(double yaw)
      {
         orientation.setToYawQuaternion(yaw);
      }

      public double getInitialYaw()
      {
         return orientation.getYaw();
      }

      public void setInitialGroundHeight(double groundHeight)
      {
         initialPosition.setZ(groundHeight);
      }

      public double getInitialGroundHeight()
      {
         return initialPosition.getZ();
      }

      public void setOffset(Vector3D additionalOffset)
      {
         initialPosition.add(additionalOffset);
      }

      public void getOffset(Vector3D offsetToPack)
      {
         offsetToPack.set(initialPosition);
      }
   }

   public class CentroidaDynamicsRobotController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      @Override
      public void initialize()
      {

      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "CentroidalDynamicsController";
      }

      @Override
      public String getDescription()
      {
         return "CentroidalRobotController";
      }

      @Override
      public void doControl()
      {

      }
   }

   public class CentroidalRobotJointMap implements JointNameMap<RobotCentroidal>
   {

      @Override
      public LegJointName[] getLegJointNames()
      {
         return null;
      }

      @Override
      public ArmJointName[] getArmJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public SpineJointName[] getSpineJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public NeckJointName[] getNeckJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getModelName()
      {
         return robotName;
      }

      @Override
      public JointRole getJointRole(String jointName)
      {
         return null;
      }

      @Override
      public NeckJointName getNeckJointName(String jointName)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public SpineJointName getSpineJointName(String jointName)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getRootBodyName()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getUnsanitizedRootJointInSdf()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getHeadName()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public boolean isTorqueVelocityLimitsEnabled()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public Set<String> getLastSimulatedJoints()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String[] getJointNamesBeforeFeet()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public RobotCentroidal[] getRobotSegments()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public RobotCentroidal getEndEffectorsRobotSegment(String jointNameBeforeEndEffector)
      {
         // TODO Auto-generated method stub
         return null;
      }
   }

   public class CentroidalRobotDescription extends RobotDescription
   {
      public CentroidalRobotDescription(String namePrefix)
      {
         super(namePrefix);
         FloatingJointDescription rootJoint = new FloatingJointDescription(namePrefix + "RootJoint");
         LinkDescription rootLink = new LinkDescription(namePrefix + "RootLink");
         LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
         rootLinkGraphics.addEllipsoid(xRadius, yRadius, zRadius, new YoAppearanceRGBColor(Color.BLUE, 0.0));
         rootLink.setLinkGraphics(rootLinkGraphics);
         rootLink.setMass(robotMass);
         rootLink.setMomentOfInertia(momentOfInertia);
         addRootJoint(rootJoint);
         rootJoint.setLink(rootLink);
      }
   }
}
