package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;



/**
 * Title:        Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */
public class KinematicPoint implements java.io.Serializable
{
   private static final long serialVersionUID = 3047881738704107434L;

   private final String name;

   // The position and velocity are in world frame. AngularVelocity is in body frame of the joint.
   private final YoFramePoint positionInWorld;
   private final YoFrameVector velocityInWorld;
   private final YoFrameVector angularVelocityInWorld;

   private final YoFrameVector offsetYoFrameVector;

   protected Joint parentJoint;

   private KinematicPointUpdater kinematicPointUpdater;

   private final YoVariableRegistry registry;

   protected final Vector3D tempVectorForOffsetFromCOM = new Vector3D(), tempVectorForWXr = new Vector3D(), tempVectorForVelocity = new Vector3D();

   private RigidBodyTransform tempTransformFromWorldToJoint = new RigidBodyTransform();
   private Vector4D offsetPlus = new Vector4D();

   private final Point3D tempPoint = new Point3D();

   public KinematicPoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public KinematicPoint(String name, Vector3DReadOnly offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, Vector3DReadOnly offset, YoVariableRegistry registry)
   {
      this.name = name;
      this.registry = registry;

      positionInWorld = new YoFramePoint(name + "_", "", ReferenceFrame.getWorldFrame(), registry);
      velocityInWorld = new YoFrameVector(name + "_d", "", ReferenceFrame.getWorldFrame(), registry);
      angularVelocityInWorld = new YoFrameVector(name + "_w", "", ReferenceFrame.getWorldFrame(), registry);

      this.offsetYoFrameVector = new YoFrameVector(name + "off", "", ReferenceFrame.getWorldFrame(), registry);
      if (offset != null)
         offsetYoFrameVector.set(offset);
   }

   public void reset()
   {
      parentJoint = null;

      offsetYoFrameVector.set(0, 0, 0);

      tempVectorForOffsetFromCOM.set(0, 0, 0);
      tempVectorForWXr.set(0, 0, 0);
      tempVectorForVelocity.set(0, 0, 0);

      tempTransformFromWorldToJoint.setIdentity();
      offsetPlus.set(0, 0, 0, 0);

      positionInWorld.set(0, 0, 0);
      velocityInWorld.set(0, 0, 0);
      angularVelocityInWorld.set(0, 0, 0);
   }

   public KinematicPointUpdater getKinematicPointUpdater()
   {
      return this.kinematicPointUpdater;
   }

   public void setKinematicPointUpdater(KinematicPointUpdater updater)
   {
      this.kinematicPointUpdater = updater;
   }

   public void setParentJoint(Joint parent)
   {
      this.parentJoint = parent;
   }

   public Joint getParentJoint()
   {
      return parentJoint;
   }

   @Override
   public String toString()
   {
      return ("name: " + name + " x: " + positionInWorld.getX() + ", y: " + positionInWorld.getY() + ", z: " + positionInWorld.getZ());
   }

   public void setOffsetJoint(double x, double y, double z)
   {
      this.offsetYoFrameVector.set(x, y, z);
   }

   public void setOffsetJoint(Vector3DReadOnly newOffset)
   {
      this.offsetYoFrameVector.set(newOffset);
   }

   public void setOffsetWorld(Tuple3DReadOnly offsetInWorld)
   {
      setOffsetWorld(offsetInWorld.getX(), offsetInWorld.getY(), offsetInWorld.getZ());
   }

   public void setOffsetWorld(double x, double y, double z)
   {
//      System.out.println("Setting offset World: " + x + ", " + y + ", " + z);
      tempTransformFromWorldToJoint.set(parentJoint.transformToNext);
      tempTransformFromWorldToJoint.invert();
      offsetPlus.set(x, y, z, 1.0);
      tempTransformFromWorldToJoint.transform(offsetPlus);

      setOffsetJoint(offsetPlus.getX(), offsetPlus.getY(), offsetPlus.getZ());
//      System.out.println("Setting offset Joint: " + offsetPlus.getX() + ", " + offsetPlus.getY() + ", " + offsetPlus.getZ());
 
      //TODO: Make sure all methods update the various variables so that a set followed by a get is consistent...
      this.positionInWorld.set(x, y, z);
   }

   public void updatePointVelocity(RotationMatrixReadOnly R0_i, Vector3DReadOnly comOffset, Vector3DReadOnly v_i, Vector3DReadOnly w_i)
   {
      this.getOffset(tempVectorForOffsetFromCOM);
      tempVectorForOffsetFromCOM.sub(comOffset);

      tempVectorForWXr.cross(w_i, tempVectorForOffsetFromCOM);
      tempVectorForVelocity.add(v_i, tempVectorForWXr);

      R0_i.transform(tempVectorForVelocity);
      velocityInWorld.set(tempVectorForVelocity);

      tempVectorForVelocity.set(w_i);
      R0_i.transform(tempVectorForVelocity);
      angularVelocityInWorld.set(tempVectorForVelocity);
   }

   protected void updatePointPosition(RigidBodyTransform t1)
   {
      if (kinematicPointUpdater != null)
      {
         // System.out.print(".");
         kinematicPointUpdater.updateKinematicPoint(this);
      }

      this.getOffset(tempPoint);
      t1.transform(tempPoint);

      positionInWorld.set(tempPoint);
   }

   public String getName()
   {
      return name;
   }

   public void getOffset(Tuple3DBasics offsetToPack)
   {
      offsetToPack.set(offsetYoFrameVector);
   }

   public Vector3D getOffsetCopy()
   {
      Vector3D ret = new Vector3D();
      getOffset(ret);

      return ret;
   }

   public double getX()
   {
      return positionInWorld.getX();
   }

   public double getY()
   {
      return positionInWorld.getY();
   }

   public double getZ()
   {
      return positionInWorld.getZ();
   }

   public double getXVelocity()
   {
      return velocityInWorld.getX();
   }

   public double getYVelocity()
   {
      return velocityInWorld.getY();
   }

   public double getZVelocity()
   {
      return velocityInWorld.getZ();
   }

   public void getPosition(Tuple3DBasics positionToPack)
   {
      positionToPack.set(positionInWorld);
   }

   public Point3D getPositionPoint()
   {
      Point3D pointToReturn = new Point3D();
      getPosition(pointToReturn);

      return pointToReturn;
   }

   public void getVelocity(Vector3DBasics velocityToPack)
   {
      velocityToPack.set(velocityInWorld);
   }

   public Vector3D getVelocityVector()
   {
      return new Vector3D(velocityInWorld);
   }

   public void setVelocity(Vector3DReadOnly velocity)
   {
      this.velocityInWorld.set(velocity);
   }

   public void getAngularVelocity(Vector3DBasics angularVelocityInWorldToPack)
   {
      angularVelocityInWorldToPack.set(this.angularVelocityInWorld);
   }

   public void setAngularVelocity(Vector3DReadOnly angularVelocityInWorld)
   {
      this.angularVelocityInWorld.set(angularVelocityInWorld);
   }

   public void setPosition(Point3DReadOnly position)
   {
      this.positionInWorld.set(position);
   }

   public YoFramePoint getYoPosition()
   {
      return positionInWorld;
   }

   public YoFrameVector getYoVelocity()
   {
      return velocityInWorld;
   }

   public YoFrameVector getYoAngularVelocity()
   {
      return angularVelocityInWorld;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
