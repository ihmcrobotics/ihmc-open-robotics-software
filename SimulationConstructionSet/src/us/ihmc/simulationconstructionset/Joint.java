package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.physics.engine.jerry.JointPhysics;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.robotics.kinematics.CommonJoint;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountInterface;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;



/**
 * Motion constraint between {@link Link Links} and physics simulation.<p>
 *
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */
public abstract class Joint implements CommonJoint, java.io.Serializable
{
   private static final long serialVersionUID = -1158152164230922246L;

   /**
    * The maximum translational acceleration this joint may undergo before throwing an
    * {@link UnreasonableAccelerationException UnreasonableAccelerationException}.
    */
   public static final double MAX_TRANS_ACCEL = 1000000000000.0;

   /**
    * The maximum rotational acceleration this joint may undergo before throwing an
    * {@link UnreasonableAccelerationException UnreasonableAccelerationException}.
    */
   public static final double MAX_ROT_ACCEL = 10000000.0;

   public Joint parentJoint;

   public final Vector3d offset = new Vector3d();    // Offset from the previous joint

   public Link link;
   protected String name;

   protected int numDOF;
   public Robot rob;

   public RigidBodyTransform transformToNext = new RigidBodyTransform();

   private final RigidBodyTransform offsetTransform3D = new RigidBodyTransform();
   public final RigidBodyTransform jointTransform3D = new RigidBodyTransform();

   protected ArrayList<CameraMount> cameraMounts;
   protected ArrayList<LidarMount> lidarMounts;
   protected ArrayList<IMUMount> imuMounts;
   protected ArrayList<WrenchCalculatorInterface> forceSensors;

   public ArrayList<Joint> childrenJoints;

   private final ArrayList<SimulatedSensor> sensors = new ArrayList<SimulatedSensor>();

   public JointPhysics<?> physics;

   /**
    * As Joint is an abstract class it is never instanced on its own.  Instead,
    * its children, FloatingJoint, FreeJoint, FloatingPlanarJoint, PinJoint, SliderJoint,
    * and NullJoint all call this superconstructor to initialize their common parameters.
    *
    * @param name Name of the new Joint.
    * @param offsetVec Vector3d representing the offset from the previous joint when all joints are at zero
    * @param rob Robot which this joint is a member of.
    * @param numDOF Number of degrees of freedom held by this joint.
    */
   protected Joint(String name, Vector3d offsetVec, Robot rob, int numDOF)
   {
      this.name = name;
      this.rob = rob;

      this.numDOF = numDOF;

      this.setOffset(offsetVec);

      this.childrenJoints = new ArrayList<Joint>();
   }

   public Joint getParentJoint()
   {
      return parentJoint;
   }

   public ArrayList<Joint> getChildrenJoints()
   {
      return childrenJoints;
   }

   public void getChildrenJoints(ArrayList<Joint> arrayListToPack)
   {
      arrayListToPack.addAll(childrenJoints);
   }

   public void recursiveGetChildrenJoints(ArrayList<Joint> arrayListToPack)
   {
      arrayListToPack.addAll(childrenJoints);

      for (Joint joint : childrenJoints)
      {
         joint.recursiveGetChildrenJoints(arrayListToPack);
      }
   }

   public void recursiveGetOneDegreeOfFreedomJoints(ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJointsToPack)
   {
      if (this instanceof OneDegreeOfFreedomJoint)
      {
         oneDegreeOfFreedomJointsToPack.add((OneDegreeOfFreedomJoint) this);
      }

      for (Joint joint : childrenJoints)
      {
         joint.recursiveGetOneDegreeOfFreedomJoints(oneDegreeOfFreedomJointsToPack);
      }
   }


   /**
    * Retrieves the joint name.
    *
    * @return Name of this joint as specified at construction.
    */
   public String getName()
   {
      return this.name;
   }

// /**
//  * Returns a VarList containing the YoVariables related to this joint.
//  * Each implementation of Joint has a different set of variables and therefore a
//  * different version of this method.
//  *
//  * @return VarList containing the variables belonging to this joint.
//  */
// protected abstract VarList getJointVars();

   /**
    * Retrieves all camera mounts at this joint.  A camera can be mounted to any of these
    * mounts and will remain attached for display purposes.  For more information see
    * {@link CameraMount Camera}.
    *
    * @return ArrayList containing all camera mounts owned by this joint.
    */
   protected ArrayList<CameraMount> getCameraMounts()
   {
      return this.cameraMounts;
   }

   protected ArrayList<LidarMount> getLidarMounts()
   {
      return this.lidarMounts;
   }

   protected ArrayList<IMUMount> getIMUMounts()
   {
      return this.imuMounts;
   }


   /**
    * Changes the offset between the current joint and its parent joint.
    *
    * @param newOffsetVector Vector3d representing this new offset.
    */
   public void changeOffsetVector(Vector3d newOffsetVector)
   {
      changeOffsetVector(newOffsetVector.getX(), newOffsetVector.getY(), newOffsetVector.getZ());
   }

   /**
    * Changes the offset between the current joint and its parent joint based on
    * the provided x, y, and z components.
    *
    * @param x Component of the new offset vector.
    * @param y Component of the new offset vector.
    * @param z Component of the new offset vector.
    */
   public void changeOffsetVector(double x, double y, double z)
   {
      this.offset.set(x, y, z);
      this.offsetTransform3D.setTranslation(this.offset);

//    this.jointGraphics3D.changeOffsetVector(offsetTransform3D);
   }

   private boolean isDynamic = true;

   /**
    * Indiciates whether or not this joint is dynamic.  By default, all joints are dynamic
    * meaning they are used during dynamics calculations.  Non dynamic root joints only deal with position
    * and velocity through Featherstone pass one.  They also store values for runge-kutta calculations.
    *
    * @return Is this joint dynamic?
    */
   public boolean isDynamic()
   {
      return this.isDynamic;
   }

   /**
    * Specify whether or not this particular joint is dynamic.  This only matters if the joint is a root joint.
    *
    * @param isDynamic Indicate whether or not the joint is dynamic, which is true by default.
    */
   public void setDynamic(boolean isDynamic)
   {
      this.isDynamic = isDynamic;
   }

   /**
    * Adds the specified GroundContactPoint to this joint.  These points allow ground contact modeling to occur
    * which provides a means of robot ground interaction.  For further examples see the tutorial.
    *
    * @param point GroundContactPoint
    * @see GroundContactPoint GroundContactPoint
    * @see GroundContactModel GroundContactModel
    */
   public void addGroundContactPoint(GroundContactPoint point)
   {
      physics.addGroundContactPoint(point);
   }

   public void addGroundContactPoint(int groupIdentifier, GroundContactPoint point)
   {
      physics.addGroundContactPoint(groupIdentifier,point);
   }

   public void addJointWrenchSensor(JointWrenchSensor jointWrenchSensor)
   {
      physics.addJointWrenchSensor(jointWrenchSensor);
   }

   public JointWrenchSensor getJointWrenchSensor()
   {
      return physics.getJointWrenchSensor();
   }

   /**
    * Adds the specified KinematicPoint to this joint.  These points allow external forces
    * and effects to be applied while also providing a means to monitor position and velocity.
    * Currently the only implementation internal to SCS is the ExternalForcePoint.
    *
    * @param point KinematicPoint to be added.
    * @see KinematicPoint KinematicPoint
    */
   public void addKinematicPoint(KinematicPoint point)
   {
      physics.addKinematicPoint(point);
   }

   /**
    * Adds the specified ExternalForcePoint.  These points allow forces to be applied to particular joints
    * allowing the creation of certain mechanical structures such as four-bar-linkages.  See the tutorial for
    * further details.
    *
    * @param point ExternalForcePoint
    * @see ExternalForcePoint ExternalForcePoint
    */
   public void addExternalForcePoint(ExternalForcePoint point)
   {
      physics.addExternalForcePoint(point);
   }

   /**
    * Returns a string representation of this joint.  This includes the joint's name, its parent joint,
    * offset vector, link, and all associated kinematic and external force points.  Once this information is displayed
    * the joint calls toString for each of it's children.
    *
    * @return String representation of this joint and its children.
    */
   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();
      Vector3d translation = new Vector3d();
      retBuffer.append("Joint: " + name + "\n");
      if (parentJoint != null)
         retBuffer.append("  Parent Joint: " + this.parentJoint.name + "\n");
      else
         retBuffer.append("  Root Joint \n");
      transformToNext.getTranslation(translation);
      retBuffer.append("   Location vector: " + translation + "\n");
      retBuffer.append("   offset vector: " + offset + "\n");

      retBuffer.append("   link: " + link);

      if( physics != null )
         retBuffer.append(physics);

      /*
       * retBuffer.append("u_i: " + u_i + "\n");
       * retBuffer.append("d_i: " + d_i + "\n");
       * retBuffer.append("w_i: " + w_i + "\n");
       * retBuffer.append("r_in: " + r_in + "\n");
       *
       * retBuffer.append("s_hat_i: " + s_hat_i + "\n");
       * retBuffer.append("Z_hat_i: " + Z_hat_i + "\n");
       * retBuffer.append("Qi_etc: " + Qi_etc + "\n");
       * retBuffer.append("Ri_h: " + Ri_h + "\n");
       * retBuffer.append("Rh_i: " + Rh_i + "\n");
       * retBuffer.append("r_i: " + r_i + "\n");
       * retBuffer.append("r_h: " + r_h + "\n");
       * retBuffer.append("I_hat_i: " + I_hat_i + "\n");
       * retBuffer.append("c_hat_i: " + c_hat_i + "\n");
       * retBuffer.append("sIs: " + sIs + "\n");
       *
       * retBuffer.append("Qi_etc: " + Qi_etc + "\n");
       */

      // retBuffer.append("qdd: " + qdd.val + "\n");
//    retBuffer.append("\n");
//
//    for (int i = 0; i < childrenJoints.size(); i++)
//    {
//       Joint nextJoint = childrenJoints.get(i);
//       retBuffer.append(nextJoint.toString());
//    }

      return retBuffer.toString();
   }

   /**
    * Update the state of this joint, each implementation handles this differently.  Graphics
    * will only be updated if specified.
    */
   protected abstract void update();


   /**
    * Recurses through each joint updating the transform between the world and it.
    * All points (kinematic, ground contact, range sensors and camera mounts)
    * are also updated.
    * The transform for each joint in the tree to the previous joint
    * is updated in this manner.  This function may update the graphics at each
    * joint if specified.
    *
    * @param tToHere Transform3D which transformToNext is based on.
    */
   protected void recursiveUpdateJoints(RigidBodyTransform tToHere, boolean updatePoints, boolean updateCameraMounts, boolean updateIMUMounts, double time)
   {
      this.update();

      if (tToHere != null)
      {
         this.transformToNext.set(tToHere);
      }
      else
      {
         this.transformToNext.setIdentity();
      }

      this.transformToNext.multiply(this.getOffsetTransform3D());
      this.transformToNext.multiply(this.getJointTransform3D());
      if (updatePoints)
         this.updatePoints(this.transformToNext, time);

      if (updateCameraMounts)
      {
         this.updateCameraMounts(this.transformToNext);
      }

      if (updateIMUMounts)
      {
         this.updateIMUMountsPositionAndVelocity(this.transformToNext);
      }

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint childJoint = childrenJoints.get(i);
         childJoint.recursiveUpdateJoints(this.transformToNext, updatePoints, updateCameraMounts, updateIMUMounts, time);
      }

   }


   protected void recursiveUpdateJointsIMUMountAccelerations()
   {
      this.updateIMUMountsAcceleration(this.transformToNext);

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint childJoint = childrenJoints.get(i);
         childJoint.recursiveUpdateJointsIMUMountAccelerations();
      }
   }


   /**
    * Retrieves the Transform3D for this joint.  This transform is used in graphics
    * and position calculations and as such is called by the recursiveUpdateJoints methods.
    * It is also used when creating a branch group copy of a specified joint.
    *
    * @return Transform3D belonging to this joint.
    */
   @Override
   public RigidBodyTransform getJointTransform3D()
   {
      return this.jointTransform3D;
   }

   protected void updateCameraMounts(RigidBodyTransform tToHere)
   {
      if (cameraMounts != null)
      {
         for (int i = 0; i < cameraMounts.size(); i++)
         {
            CameraMount mount = cameraMounts.get(i);
            mount.updateTransform(tToHere);
         }
      }
   }

   protected void updateLidarMounts(RigidBodyTransform tToHere)
   {
      if (lidarMounts != null)
      {
         for (int i = 0; i < lidarMounts.size(); i++)
         {
            LidarMount mount = lidarMounts.get(i);
            mount.updateTransform(tToHere, this.rob.getTime());
         }
      }
   }

   protected void updateIMUMountsPositionAndVelocity(RigidBodyTransform tToHere)
   {
      if (imuMounts != null)
      {
         for (int i = 0; i < imuMounts.size(); i++)
         {
            IMUMount mount = imuMounts.get(i);
            mount.updateIMUMountPositionAndVelocity();
         }
      }
   }

   protected void updateIMUMountsAcceleration(RigidBodyTransform tToHere)
   {
      if (imuMounts != null)
      {
         for (int i = 0; i < imuMounts.size(); i++)
         {
            IMUMount mount = imuMounts.get(i);
            mount.updateIMUMountAcceleration();
         }
      }
   }

   private void updateSensorMounts(RigidBodyTransform tToHere, double time)
   {
      for (int i = 0; i < sensors.size(); i++)
      {
         SimulatedSensor simulatedSensor = sensors.get(i);
         simulatedSensor.updateTransform(tToHere, time);
      }
   }

   /**
    * Updates the transforms for each point type based on a new parent transform.
    * This function is called whenever the joints are updated.
    *
    * @param tToHere New transform to target space, this is to be used to update
    * the transform of each point.
    * @param time
    */

   protected void updatePoints(RigidBodyTransform tToHere, double time)
   {
      updateSensorMounts(tToHere, time);

      // TODO Rip out and move to physics engine
      // +++JEP OPTIMIZE
      if (physics.groundContactPointGroupList != null)
      {
         for (int i = 0; i < physics.groundContactPointGroupList.size(); i++)
         {
            ArrayList<GroundContactPoint> groundContactPoints = physics.groundContactPointGroupList.get(i).getGroundContactPoints();
            for (int y = 0; y < groundContactPoints.size(); y++)
            {
               GroundContactPoint childPoint = groundContactPoints.get(y);
               childPoint.updatePointPosition(tToHere);
            }
         }

         /*
          *  // Update the velocities of the points not in contact, just in case the user is using them
          * // Those in contact will be updated by featherstonePassOne.
          * // Now update the points attached to the joint:
          * R0_i.set(Ri_0);
          * R0_i.transpose();
          *
          * ArrayList groundContactPointsNotInContact = groundContactPointGroup.getGroundContactPointsNotInContact();
          * for (int i = 0; i < groundContactPointsNotInContact.size(); i++)
          * {
          *  GroundContactPoint point = (GroundContactPoint) groundContactPointsNotInContact.get(i);
          *  point.updatePointVelocity(R0_i, this.link.comOffset, v_i, w_i);
          * }
          */
      }

      if (physics.kinematicPoints != null)
      {
         for (int i = 0; i < physics.kinematicPoints.size(); i++)
         {
            KinematicPoint childPoint = physics.kinematicPoints.get(i);
            childPoint.updatePointPosition(tToHere);
         }
      }
   }

   /**
    * Adds the specified joint as a child of this joint.  This function also handles
    * all setup necessary for back tracking from child to parent.  Each joint must have
    * an associated link before being added.
    *
    * @param childJoint Child joint to be added.
    */
   public void addJoint(Joint childJoint)
   {
      childJoint.parentJoint = this;    // Set his parent to me for later back tracking...

      childrenJoints.add(childJoint);
   }


   /**
    * Retrieves the transform describing the translation between the current joint and the
    * previous joint.  This transform is translation only, with all but the last column
    * mimicking the identity matrix.
    *
    * @return Transform3D describing the translation between this joint and its parent.
    */
   @Override
   public RigidBodyTransform getOffsetTransform3D()
   {
      return this.offsetTransform3D;
   }

   /**
    * Adds the specified camera mount to this joint.  A single joint may contain multiple mounts.
    *
    * @param mount CameraMount to be added.
    * @see CameraMount CameraMount
    */
   public void addCameraMount(CameraMount mount)
   {
      if (cameraMounts == null)
         cameraMounts = new ArrayList<CameraMount>();
      cameraMounts.add(mount);
      mount.setParentJoint(this);
   }

   public void addLidarMount(LidarMount mount)
   {
      if (lidarMounts == null)
         lidarMounts = new ArrayList<LidarMount>();
      lidarMounts.add(mount);
      mount.setParentJoint(this);
   }

   /**
    * Adds the specified imu mount to this joint.  A single joint may contain multiple mounts.
    *
    * @param mount IMUMount to be added.
    * @see IMUMount IMUMount
    */
   public void addIMUMount(IMUMount mount)
   {
      if (imuMounts == null)
         imuMounts = new ArrayList<IMUMount>();
      imuMounts.add(mount);
      mount.setParentJoint(this);
   }

   /**
    * Adds the specified force sensor to this joint.
    *
    * @param forceSensor forceSensor to add
    */
   public void addForceSensor(WrenchCalculatorInterface forceSensor)
   {
      if(forceSensors == null)
         forceSensors = new ArrayList<WrenchCalculatorInterface>();

      forceSensors.add(forceSensor);
   }

   /**
    * Retrieves the link associated with this joint.  Every joint has a member link which handels
    * the physical and graphical properties of the space between this joint and its children.  This includes
    * the link mass and related properties.
    *
    * @return The link belonging to this joint.
    */
   public Link getLink()
   {
      return this.link;
   }

   // protected void setName(String n){this.name=n;}
   // protected String getName(){return this.name;}


   /**
    * Retrieves the number of degrees of freedom held by this joint.  Each degree
    * of freedom represents the number of axis about which a joint may freely move.
    * Pin and slider joints have only one degree of freedom, while free joints have six.
    *
    * @return This joints degree of freedom.
    */
   protected int getNumDOF()
   {
      return this.numDOF;
   }

   // protected VarList getVarList(){return this.jointVars;}

   /**
    * Sets a new offset vector between this joint and its parent.  This function
    * is used to initialize the offset vector in the constructor.  It should not
    * be employed to modify this vector after creation as it does not inform the
    * parent joint of the change.
    *
    * @param offset New offset vector.
    */
   private void setOffset(Vector3d offset)
   {
      this.offset.set(offset);
      this.offsetTransform3D.setTranslation(offset);
   }

   public void getOffset(Vector3d offsetToPack)
   {
      offsetToPack.set(offset);
   }



   /**
    * Sets the link for this joint.  If a previous link existed it is replaced, removing all graphics.
    *
    * @param newLink New link for this joint.
    */
   public void setLink(Link newLink)
   {
      this.link = newLink;
      this.link.setParentJoint(this);
   }

   /**
    * Recurse over the children of this joint and add their CameraMounts to the
    * provided ArrayList.
    *
    * @param list ArrayList to which the CameraMounts are added.
    */
   protected void recursiveGetCameraMounts(ArrayList<CameraMountInterface> list)
   {
      if (cameraMounts != null)
         list.addAll(this.cameraMounts);

      // Recurse over the children:

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint child = childrenJoints.get(i);
         child.recursiveGetCameraMounts(list);
      }
   }

   protected void recursiveGetLidarMounts(ArrayList<LidarMount> list)
   {
      if (lidarMounts != null)
         list.addAll(this.lidarMounts);

      // Recurse over the children:

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint child = childrenJoints.get(i);
         child.recursiveGetLidarMounts(list);
      }
   }

   protected void recursiveGetForceSensors(ArrayList<WrenchCalculatorInterface> list)
   {
      if (forceSensors != null)
      {
         list.addAll(forceSensors);
      }
      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint child = childrenJoints.get(i);
         child.recursiveGetForceSensors(list);
      }
   }

   protected void recursiveGetIMUMounts(ArrayList<IMUMount> list)
   {
      if (imuMounts != null)
         list.addAll(this.imuMounts);

      // Recurse over the children:

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint child = childrenJoints.get(i);
         child.recursiveGetIMUMounts(list);
      }
   }

   protected void recursiveGetSensors(ArrayList<SimulatedSensor> simulatedSensorsToPack)
   {
      getSensors(simulatedSensorsToPack);

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         Joint child = childrenJoints.get(i);
         child.recursiveGetSensors(simulatedSensorsToPack);
      }
   }



   private void getSensors(ArrayList<SimulatedSensor> simulatedSensorsToPack)
   {
      simulatedSensorsToPack.addAll(sensors);
   }

   // TODO: GT: Test SimulatedSensors
   public void addSensor(SimulatedSensor simulatedSensor)
   {
      sensors.add(simulatedSensor);
   }

   public void recursiveGetAllGroundContactPoints(ArrayList<GroundContactPoint> groundContactPoints)
   {
      physics.recursiveGetAllGroundContactPoints(groundContactPoints);
   }

   /**
    * Sets ret to the transform between world space and this joint space.
    *
    * @param ret Transform3D
    */
   public void getTransformToWorld(RigidBodyTransform ret)
   {
      ret.set(transformToNext);
   }

   /**
    * Sets rotation and translation to the rotational and translational
    * components of the the transform between world space and this joint space.
    *
    * @param rotation Quat4d representation of the rotational component.
    * @param translation Vector3d representation of the traslational component.
    */
   public void getTransformToWorld(Quat4d rotation, Vector3d translation)
   {
      transformToNext.get(rotation, translation);
   }

   /**
    * Retrieves the rotational component of the transform between world space and
    * this joint space.
    *
    * @param rotation Matrix3d containing the rotational component.
    */
   public void getRotationToWorld(Matrix3d rotation)
   {
      transformToNext.getRotation(rotation);
   }

   /**
    * Retrieves the quaternion representation of the rotational component of the
    * transform between world space and this joint space.
    *
    * @param rotation Quat4d to store the rotational transform.
    */
   public void getRotationToWorld(Quat4d rotation)
   {
      transformToNext.getRotation(rotation);
   }

   /**
    * Retrieves the translational component of the transform between world space
    * and this joint space in vector form.
    *
    * @param translation Vector3d representing the translation between world and joint space.
    */
   public void getTranslationToWorld(Vector3d translation)
   {
      transformToNext.getTranslation(translation);
   }

   private Vector3d tempVector3d = new Vector3d();
   private Quat4d tempQuat4d = new Quat4d();

   /**
    * Stores the x, y, and z components of the translation between world space and joint space
    * in the provided YoVariables.
    *
    * @param x YoVariable to store the x component.
    * @param y YoVariable to store the y component.
    * @param z YoVariable to store the z component.
    */
   public void getXYZToWorld(DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable z)
   {
      getTranslationToWorld(tempVector3d);
      x.set(tempVector3d.getX());
      y.set(tempVector3d.getY());
      z.set(tempVector3d.getZ());
   }

   /**
    * Retrieves the rotation between world space and joint space in terms of yaw, pitch
    * and roll.  These values are stored in the provided YoVariables.
    *
    * @param yaw YoVariable to store yaw.
    * @param pitch YoVariable to store pitch.
    * @param roll YoVariable to store roll.
    */
   public void getYawPitchRollToWorld(DoubleYoVariable yaw, DoubleYoVariable pitch, DoubleYoVariable roll)
   {
      getRotationToWorld(tempQuat4d);
      double q_x = tempQuat4d.getX(), q_y = tempQuat4d.getY(), q_z = tempQuat4d.getZ(), q_w = tempQuat4d.getW();

      yaw.set(Math.atan2(2.0 * q_x * q_y + 2.0 * q_z * q_w, 1.0 - 2.0 * q_y * q_y - 2.0 * q_z * q_z));
      pitch.set(Math.asin(-2.0 * q_x * q_z + 2.0 * q_w * q_y));
      roll.set(Math.atan2(2.0 * q_y * q_z + 2.0 * q_x * q_w, 1.0 - 2.0 * q_x * q_x - 2.0 * q_y * q_y));
   }

   // TODO: Rename this to match whatever it is that it does.
   public double[] get3DRotation()
   {
      double[] rotation = new double[3];

      getRotationToWorld(tempQuat4d);
      double q_x = tempQuat4d.getX(), q_y = tempQuat4d.getY(), q_z = tempQuat4d.getZ(), q_w = tempQuat4d.getW();

      rotation[2] = Math.atan2(2.0 * q_x * q_y + 2.0 * q_z * q_w, 1.0 - 2.0 * q_y * q_y - 2.0 * q_z * q_z);
      rotation[1] = Math.asin(-2.0 * q_x * q_z + 2.0 * q_w * q_y);
      rotation[0] = Math.atan2(2.0 * q_y * q_z + 2.0 * q_x * q_w, 1.0 - 2.0 * q_x * q_x - 2.0 * q_y * q_y);

      return rotation;
   }

   public Link getLink(String linkName)
   {
      if (this.link.getName().equals(linkName)) return this.link;

      for (Joint childJoint : childrenJoints)
      {
         Link link = childJoint.getLink(linkName);
         if (link != null) return link;
      }

      return null;
   }

   public void removeChildJoint(Joint jointToRemove)
   {
      boolean removed = childrenJoints.remove(jointToRemove);
      if (!removed) throw new RuntimeException("Could not remove joint. Joint " + jointToRemove.getName() + " was not a child of joint " + this.getName());

      jointToRemove.parentJoint = null;
   }


   public Robot getRobot()
   {
      return rob;
   }

   public void getJointAxis(Vector3d axisToPack)
   {
      physics.getJointAxis(axisToPack);
   }

   public void removeExternalForcePoint(ExternalForcePoint externalForcePoint)
   {
      physics.removeExternalForcePoint(externalForcePoint);
   }

   public GroundContactPointGroup getGroundContactPointGroup()
   {
      return physics.getGroundContactPointGroup();
   }

   public GroundContactPointGroup getGroundContactPointGroup(int groupIdentifier)
   {
      return physics.getGroundContactPointGroup(groupIdentifier);
   }

   public ArrayList<ExternalForcePoint> getExternalForcePoints()
   {
      return physics.getExternalForcePoints();
   }
}