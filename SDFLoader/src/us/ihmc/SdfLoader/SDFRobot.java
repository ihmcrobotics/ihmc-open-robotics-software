package us.ihmc.SdfLoader;

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
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.polarLidarGeometry.LIDARScanDefinition;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanDefinition;
import us.ihmc.utilities.screwTheory.RigidBodyInertia;

import com.yobotics.simulationconstructionset.CameraMount;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SliderJoint;
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
   private static final boolean USE_POLAR_LIDAR_MODEL = true;
   private static final boolean SHOW_COM_REFERENCE_FRAMES = false;

   private static final long serialVersionUID = 5864358637898048080L;

   private final ArrayList<String> resourceDirectories;
   
   private final HashMap<String, OneDegreeOfFreedomJoint> oneDoFJoints = new HashMap<String, OneDegreeOfFreedomJoint>();
   private final HashMap<String, Transform3D> jointTransforms = new HashMap<String, Transform3D>();
   

   private final FloatingJoint rootJoint;

   private final SideDependentList<ArrayList<GroundContactPoint>> footGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

   private final HashMap<String, SDFCamera> cameras = new HashMap<String, SDFCamera>();

   public SDFRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap)
   {
      super(generalizedSDFRobotModel.getName());
      this.resourceDirectories = generalizedSDFRobotModel.getResourceDirectories();

      System.out.println("Creating root joints for root links");

      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();

      if (rootLinks.size() > 1)
      {
         throw new RuntimeException("Can only accomodate one root link for now");
      }

      SDFLinkHolder rootLink = rootLinks.get(0);
      
      Vector3d offset = new Vector3d();
      generalizedSDFRobotModel.getTransformToRoot().get(offset);
      rootJoint = new FloatingJoint(rootLink.getName(), offset, this);
      Link scsRootLink = createLink(rootLink, new Transform3D());
      rootJoint.setLink(scsRootLink);
      addRootJoint(rootJoint);


      for (SDFJointHolder child : rootLink.getChildren())
      {
         addJointsRecursively(child, rootJoint, MatrixTools.IDENTITY);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         footGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
      }

      HashMap<String, Integer> counters = new HashMap<String, Integer>();
      if (sdfJointNameMap != null)
      {
         for (Pair<String, Vector3d> jointContactPoint : sdfJointNameMap.getJointNameGroundContactPointMap())
         {
            String jointName = jointContactPoint.first();

            int count;
            if (counters.get(jointName) == null)
               count = 0;
            else
               count = counters.get(jointName);

            Vector3d gcOffset = jointContactPoint.second();
            jointTransforms.get(jointName).transform(gcOffset);
            
            
            GroundContactPoint groundContactPoint = new GroundContactPoint("gc_" + SDFJointHolder.createValidVariableName(jointName) + "_" + count++,
                  gcOffset, this);
            
            ExternalForcePoint externalForcePoint = new ExternalForcePoint("ef_" + SDFJointHolder.createValidVariableName(jointName) + "_" + count++,
                  gcOffset, this);
            
            oneDoFJoints.get(jointName).addGroundContactPoint(groundContactPoint);
            oneDoFJoints.get(jointName).addExternalForcePoint(externalForcePoint);

            counters.put(jointName, count);

            if (SHOW_CONTACT_POINTS)
            {
               Graphics3DObject graphics = oneDoFJoints.get(jointName).getLink().getLinkGraphics();
               graphics.identity();
               graphics.translate(jointContactPoint.second());
               double radius = 0.01;
               graphics.addSphere(radius, YoAppearance.Orange());
            }

            for (RobotSide robotSide : RobotSide.values())
            {
               if (jointName.equals(sdfJointNameMap.getJointBeforeFootName(robotSide)))
                  footGroundContactPoints.get(robotSide).add(groundContactPoint);
            }
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

   public OneDegreeOfFreedomJoint getOneDoFJoint(String name)
   {
      return oneDoFJoints.get(name);
   }

   private void addJointsRecursively(SDFJointHolder joint, Joint scsParentJoint, Matrix3d chainRotationIn)
   {
      Matrix3d rotation = new Matrix3d();
      Vector3d offset = new Vector3d();
      joint.getTransformToParentJoint().get(rotation, offset);
      Vector3d jointAxis = new Vector3d(joint.getAxis());

      Matrix3d chainRotation = new Matrix3d(chainRotationIn);
      chainRotation.mul(rotation);

    
      Transform3D rotationTransform = new Transform3D();
      rotationTransform.setRotation(chainRotationIn);
      rotationTransform.transform(offset);

      
      
      Transform3D jointTransform = new Transform3D();
      jointTransform.setRotation(chainRotation);
      jointTransforms.put(joint.getName(), jointTransform);
      
      
      
      String sanitizedJointName = SDFConversionsHelper.sanitizeJointName(joint.getName());
      
      Joint scsJoint;
      switch (joint.getType())
      {
      case REVOLUTE:
         PinJoint pinJoint = new PinJoint(sanitizedJointName, offset, this, jointAxis);
         if(joint.hasLimits())
         {
            if ((joint.getContactKd() == 0.0) && (joint.getContactKp() == 0.0))
            {
               if (pinJoint.getName().contains("f0") || pinJoint.getName().contains("f1") || pinJoint.getName().contains("f2") || pinJoint.getName().contains("f3"))
               {
                  pinJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 10.0, 2.5);
               }
               else
               {
                  pinJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 100.0, 20.0);
               }
            }
            else
            {
               pinJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 0.0001 * joint.getContactKp(), joint.getContactKd());
            }
         }
         
         pinJoint.setDamping(joint.getDamping());
         
         oneDoFJoints.put(joint.getName(), pinJoint);
         scsJoint = pinJoint;
         break;
      case PRISMATIC:
         SliderJoint sliderJoint = new SliderJoint(sanitizedJointName, offset, this, jointAxis);
         if(joint.hasLimits())
         {
            if ((joint.getContactKd() == 0.0) && (joint.getContactKp() == 0.0))
            {
               sliderJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 100.0, 20.0);
            }
            else
            {
               sliderJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 0.0001 * joint.getContactKp(), joint.getContactKd());
            }
         }
         sliderJoint.setDamping(joint.getDamping());
         oneDoFJoints.put(joint.getName(), sliderJoint);
         scsJoint = sliderJoint;
         break;
      default:
         throw new RuntimeException("Joint type not implemented: " + joint.getType());
      }
      
      scsJoint.setLink(createLink(joint.getChild(), rotationTransform));
      scsParentJoint.addJoint(scsJoint);

      if (DEBUG)
         if ("hokuyo_joint".equals(scsJoint.getName()))
            System.out.println("hokuyo joint's parent is : " + scsParentJoint.getName());

      addCameraMounts(scsJoint, joint.getChild());
      addLidarMounts(scsJoint, joint.getChild());
      

      

      for (SDFJointHolder child : joint.getChild().getChildren())
      {
         addJointsRecursively(child, scsJoint, chainRotation);
      }

   }

   private void addCameraMounts(Joint scsJoint, SDFLinkHolder child)
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

   private void addLidarMounts(Joint scsJoint, SDFLinkHolder child)
   {
      if (child.getSensors() != null)
      {
         for (SDFSensor sensor : child.getSensors())
         {
            if ("ray".equals(sensor.getType()))
            {
               if (DEBUG)
                  System.out.println("SDFRobot has a lidar!");
               if (DEBUG)
                  System.out.println("SDFRobot: the lidar is attached to link: " + scsJoint.getName());
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

//                double sdfAngularResolution = Double.parseDouble(sdfHorizontalScan.getSillyAndProbablyNotUsefulResolution());
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

                  PolarLidarScanDefinition polarDefinition = new PolarLidarScanDefinition(sdfSamples, 1, (float) sdfMaxAngle, (float) sdfMinAngle, 0.0f, 0.0f,
                                                                (float) sdfMinRange);
                  LIDARScanDefinition lidarScanDefinition = LIDARScanDefinition.PlanarSweep(sdfMaxAngle - sdfMinAngle, sdfSamples);
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

//                updateParameters.setServerPort() We can't know the server port in SDF Uploaders, so this must be specified afterwords, but searching the robot tree and assigning numbers.

                  if (!USE_POLAR_LIDAR_MODEL)
                  {
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

   private Link createLink(SDFLinkHolder link, Transform3D rotationTransform)
   {

      Link scsLink = new Link(link.getName());
      if(link.getVisuals() != null)
      {
         SDFGraphics3DObject linkGraphics = new SDFGraphics3DObject(link.getVisuals(), resourceDirectories, rotationTransform);
         scsLink.setLinkGraphics(linkGraphics);
      }
      
      
      
      double mass = link.getMass();
      Matrix3d inertia = link.getInertia();
      Vector3d CoMOffset = link.getCoMOffset();
      
      
      RigidBodyInertia rigidBodyInertia = new RigidBodyInertia(ReferenceFrame.getWorldFrame(), inertia, mass);
      ReferenceFrame jointFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("toroidFrame", ReferenceFrame.getWorldFrame(), rotationTransform);
      rigidBodyInertia.changeFrame(jointFrame);
      
      rotationTransform.transform(CoMOffset);

      
      scsLink.setComOffset(CoMOffset);
      scsLink.setMass(mass);
      scsLink.setMomentOfInertia(rigidBodyInertia.getMassMomentOfInertiaPartCopy());
      if(SHOW_COM_REFERENCE_FRAMES)
      {
         scsLink.addCoordinateSystemToCOM(0.1);
      }
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

      return oneDoFJoints.get(name).getLink().getLinkGraphics();
   }

   public SDFCamera getCamera(String name)
   {
      return cameras.get(name);
   }

   public List<GroundContactPoint> getFootGroundContactPoints(RobotSide robotSide)
   {
      return footGroundContactPoints.get(robotSide);
   }
}
