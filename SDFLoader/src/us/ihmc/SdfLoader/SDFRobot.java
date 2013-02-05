package us.ihmc.SdfLoader;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Camera;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Plugin;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Range;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan.HorizontalScan;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidRobot;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.polarLidarGeometry.LIDARScanDefinition;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanDefinition;

import com.yobotics.simulationconstructionset.CameraMount;
import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.graphics.GraphicsObjectsHolder;
import com.yobotics.simulationconstructionset.simulatedSensors.FastPolarRayCastLIDAR;
import com.yobotics.simulationconstructionset.simulatedSensors.RayTraceLIDARSensor;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorLimitationParameters;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorNoiseParameters;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

public class SDFRobot extends Robot implements GraphicsObjectsHolder, HumanoidRobot    // TODO: make an SDFHumanoidRobot
{
   private static final boolean DEBUG = true;
   private static final boolean SHOW_CONTACT_POINTS = true;
   private static final boolean USE_POLAR_LIDAR_MODEL  = true;

   private static final long serialVersionUID = 5864358637898048080L;
   
   private final File resourceDirectory;
   private final HashMap<String, PinJoint> robotJoints = new HashMap<String, PinJoint>();

   private final FloatingJoint rootJoint;

   private final SideDependentList<ArrayList<GroundContactPoint>> groundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

   private final HashMap<String, SDFCamera> cameras = new HashMap<String, SDFCamera>();

   public SDFRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap)
   {
      super(generalizedSDFRobotModel.getName());
      this.resourceDirectory = generalizedSDFRobotModel.getResourceDirectory();

      System.out.println("Creating root joints for root links");

      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();

      if (rootLinks.size() > 1)
      {
         throw new RuntimeException("Can only accomodate one root link for now");
      }

      SDFLinkHolder rootLink = rootLinks.get(0);

      rootJoint = new FloatingJoint(rootLink.getName(), generalizedSDFRobotModel.getTransformToRoot(), this);
      Link scsRootLink = createLink(rootLink);
      rootJoint.setLink(scsRootLink);
      addRootJoint(rootJoint);

      
      Matrix3d rootRotation = new Matrix3d();
      generalizedSDFRobotModel.getTransformToRoot().get(rootRotation);
      for (SDFJointHolder child : rootLink.getChildren())
      {
         addJointsRecursively(child, rootJoint, rootRotation);
      }

      if (sdfJointNameMap != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ArrayList<GroundContactPoint> groundContactPointsForSide = new ArrayList<GroundContactPoint>();

            if (sdfJointNameMap.getJointGroundContactPoints(robotSide) != null)
            {
               int i = 0;
               for (Pair<String,Vector3d> jointContactPoint : sdfJointNameMap.getJointGroundContactPoints(robotSide))
               {
                  String jointName = jointContactPoint.first();
                  GroundContactPoint groundContactPoint = new GroundContactPoint("gc_" + SDFJointHolder.createValidVariableName(jointName) + "_" + i, jointContactPoint.second(), this);
                  robotJoints.get(jointName).addGroundContactPoint(groundContactPoint);
                  groundContactPointsForSide.add(groundContactPoint);
   
                  if (SHOW_CONTACT_POINTS)
                  {
                     Graphics3DObject graphics = robotJoints.get(jointName).getLink().getLinkGraphics();
                     graphics.identity();
                     graphics.translate(jointContactPoint.second());
                     graphics.addSphere(0.002, YoAppearance.Orange());
                  }
                  i++;
               }
            }
            groundContactPoints.put(robotSide, groundContactPointsForSide);
         }
         
      }

      Point3d centerOfMass = new Point3d();
      double totalMass = computeCenterOfMass(centerOfMass);
      System.out.println("SDFRobot: Total robot mass: " + totalMass);

   }

   public void getRootJointToWorldTransform(Transform3D transform)
   {
      rootJoint.getTransformToWorld(transform);
   }

   public void setPositionInWorld(Vector3d offset)
   {
      rootJoint.setPosition(offset);
   }

   public void setOrientation(double yaw, double pitch, double roll)
   {
      rootJoint.setYawPitchRoll(yaw, pitch, roll);
   }
   
   public PinJoint getPinJoint(String name)
   {
      return robotJoints.get(name);
   }

   private void addJointsRecursively(SDFJointHolder joint, Joint scsParentJoint, Matrix3d chainRotationIn)
   {
      Matrix3d rotation = new Matrix3d();
      joint.getTransformToParentJoint().get(rotation);

      Matrix3d chainRotation = new Matrix3d(chainRotationIn);
      chainRotation.mul(rotation);
      
      Transform3D orientationTransform = new Transform3D();
      orientationTransform.set(chainRotation);
      orientationTransform.invert();
      
      Vector3d jointAxis = new Vector3d(joint.getAxis());
      orientationTransform.transform(jointAxis);
      
      PinJoint scsJoint = new PinJoint(SDFConversionsHelper.sanitizeJointName(joint.getName()), joint.getTransformToParentJoint(), this, jointAxis);
      scsJoint.setLink(createLink(joint.getChild()));
      scsParentJoint.addJoint(scsJoint);

      if(DEBUG)
         if("hokuyo_joint".equals(scsJoint.getName()))
            System.out.println("hokuyo joint's parent is : "+ scsParentJoint.getName());
      
      addCameraMounts(scsJoint, joint.getChild());
      addLidarMounts(scsJoint, joint.getChild());

      if ((joint.getContactKd() == 0.0) && (joint.getContactKp() == 0.0))
      {
         if (scsJoint.getName().contains("finger"))
         {
            scsJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 10.0, 2.5);
         }
         else
         {
            scsJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 100.0, 20.0);
         }
      }
      else
      {
         scsJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 0.0001 * joint.getContactKp(), joint.getContactKd());
      }

      robotJoints.put(joint.getName(), scsJoint);

      for (SDFJointHolder child : joint.getChild().getChildren())
      {
         addJointsRecursively(child, scsJoint, chainRotation);
      }

   }

   private void addCameraMounts(PinJoint scsJoint, SDFLinkHolder child)
   {
      if (child.getSensors() != null)
      {
         for (SDFSensor sensor : child.getSensors())
         {
            if ("camera".equals(sensor.getType()) || "multicamera".equals(sensor.getType()))
            {
               // TODO: handle left and right sides of multicamera
               final Camera camera = sensor.getCamera();

               if (camera != null)
               {
                  Transform3D pose = SDFConversionsHelper.poseToTransform(sensor.getPose());
                  double fieldOfView = Double.parseDouble(camera.getHorizontalFov());
                  double clipNear = Double.parseDouble(camera.getClip().getNear());
                  double clipFar = Double.parseDouble(camera.getClip().getFar());
                  CameraMount mount = new CameraMount(sensor.getName(), pose, fieldOfView, clipNear, clipFar, this);
                  scsJoint.addCameraMount(mount);

                  SDFCamera sdfCamera = new SDFCamera(Integer.parseInt(camera.getImage().getWidth()), Integer.parseInt(camera.getImage().getHeight()));
                  cameras.put(sensor.getName(), sdfCamera);
               }
               else
               {
                  System.err.println("JAXB loader: No camera section defined for camera sensor " + sensor.getName() + ", ignoring sensor.");
               }
            }

         }
      }
   }

   private void addLidarMounts(PinJoint scsJoint, SDFLinkHolder child)
   {
      if (child.getSensors() != null)
      {
         for (SDFSensor sensor : child.getSensors())
         {
            if ("ray".equals(sensor.getType()))
            {
               if (DEBUG)
                  System.out.println("SDFRobot has a lidar!");
               if(DEBUG)
                  System.out.println("SDFRobot: the lidar is attached to link: "+scsJoint.getName());
               Ray sdfRay = sensor.getRay();
               if (sdfRay == null)
               {
                  System.err.println("SDFRobot: lidar not present in ray type sensor " + sensor.getName() + ". Ignoring this sensor.");
               }
               else
               {
                  Range sdfRange = sdfRay.getRange();
                  Scan sdfScan = sdfRay.getScan();
                  double sdfMaxRange = Double.parseDouble(sdfRange.getMax());
                  double sdfMinRange = Double.parseDouble(sdfRange.getMin());
                  HorizontalScan sdfHorizontalScan = sdfScan.getHorizontal();
                  double sdfMaxAngle = Double.parseDouble(sdfHorizontalScan.getMaxAngle());
                  double sdfMinAngle = Double.parseDouble(sdfHorizontalScan.getMinAngle());
//                  double sdfAngularResolution = Double.parseDouble(sdfHorizontalScan.getSillyAndProbablyNotUsefulResolution());
                  int sdfSamples = Integer.parseInt(sdfHorizontalScan.getSamples());
                  double sdfRangeResolution = Double.parseDouble(sdfRay.getRange().getResolution());

                  double sdfGaussianNoise = 0.0;
                  boolean sdfAlwaysOn = true;
                  double sdfUpdateRate = Double.parseDouble(sensor.getUpdateRate());
                  Plugin sdfLidarPlugin = sensor.getPlugin();
                  if (sdfLidarPlugin != null)
                  {
                     sdfGaussianNoise = Double.parseDouble(sdfLidarPlugin.getGaussianNoise());
                     sdfAlwaysOn = Boolean.parseBoolean(sdfLidarPlugin.getAlwaysOn());
                     sdfUpdateRate = Double.parseDouble(sdfLidarPlugin.getUpdateRate());
                  }
                  else
                  {
                     System.err.println("SDFRobot: lidar does not have associated plugin in sensor " + sensor.getName() + ". Assuming zero gaussian noise.");
                  }

                  PolarLidarScanDefinition polarDefinition = new PolarLidarScanDefinition(sdfSamples, 1, (float)sdfMaxAngle, (float)sdfMinAngle, 0.0f, 0.0f, (float)sdfMinRange);
                  LIDARScanDefinition lidarScanDefinition = LIDARScanDefinition.PlanarSweep(sdfMaxAngle-sdfMinAngle, sdfSamples);
                  Transform3D transform3d = SDFConversionsHelper.poseToTransform(sensor.getPose());
                  
                  SimulatedLIDARSensorNoiseParameters noiseParameters = new SimulatedLIDARSensorNoiseParameters();
                  noiseParameters.setGaussianNoiseStandardDeviation(sdfGaussianNoise);
                  
                  SimulatedLIDARSensorLimitationParameters limitationParameters = new SimulatedLIDARSensorLimitationParameters();
                  limitationParameters.setMaxRange(sdfMaxRange);
                  limitationParameters.setMinRange(sdfMinRange);
                  limitationParameters.setQuantization(sdfRangeResolution);
                  
                  SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
                  updateParameters.setAlwaysOn(sdfAlwaysOn);
                  updateParameters.setUpdateRate(sdfUpdateRate);
//                  updateParameters.setServerPort() We can't know the server port in SDF Uploaders, so this must be specified afterwords, but searching the robot tree and assigning numbers.

                  if (!USE_POLAR_LIDAR_MODEL){
                     RayTraceLIDARSensor scsLidar = new RayTraceLIDARSensor(transform3d, lidarScanDefinition);
                     scsLidar.setNoiseParameters(noiseParameters);
                     scsLidar.setSensorLimitationParameters(limitationParameters);
                     scsLidar.setLidarDaemonParameters(updateParameters);
                     scsJoint.addSensor(scsLidar);
                  }
                  else
                  {
                     FastPolarRayCastLIDAR scsLidar = new FastPolarRayCastLIDAR(transform3d, polarDefinition);
                     scsLidar.setNoiseParameters(noiseParameters);
                     scsLidar.setSensorLimitationParameters(limitationParameters);
                     scsLidar.setLidarDaemonParameters(updateParameters);
                     scsJoint.addSensor(scsLidar);
                  }

               }
            }

         }
      }
   }

   private Link createLink(SDFLinkHolder link)
   {
      SDFGraphics3DObject linkGraphics = new SDFGraphics3DObject(link.getVisuals(), resourceDirectory);

      Link scsLink = new Link(link.getName());
      scsLink.setLinkGraphics(linkGraphics);
      scsLink.setComOffset(link.getCoMOffset());
      scsLink.setMass(link.getMass());
      scsLink.setMomentOfInertia(link.getInertia());

      return scsLink;

   }

   public FrameVector getRootJointVelocity()
   {
      FrameVector ret = new FrameVector(ReferenceFrame.getWorldFrame());
      rootJoint.getVelocity(ret.getVector());

      return ret;
   }

   public FrameVector getPelvisAngularVelocityInPelvisFrame(ReferenceFrame pelvisFrame)
   {
      Vector3d angularVelocity = rootJoint.getAngularVelocityInBody();

      return new FrameVector(pelvisFrame, angularVelocity);
   }

   public Graphics3DObject getGraphicsObject(String name)
   {
      if (rootJoint.getName().equals(name))
      {
         return rootJoint.getLink().getLinkGraphics();
      }

      return robotJoints.get(name).getLink().getLinkGraphics();
   }

   public SDFCamera getCamera(String name)
   {
      return cameras.get(name);
   }

   public List<GroundContactPoint> getFootGroundContactPoints(RobotSide robotSide)
   {
      return groundContactPoints.get(robotSide);
   }
}
