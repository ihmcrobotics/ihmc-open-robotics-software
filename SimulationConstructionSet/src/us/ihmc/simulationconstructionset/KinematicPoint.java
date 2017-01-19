package us.ihmc.simulationconstructionset;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;



/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */
public class KinematicPoint implements java.io.Serializable
{
   private static final long serialVersionUID = 3047881738704107434L;

   public final String name;

   private final Point3d temp = new Point3d();

   private final YoFramePoint position;
   private final YoFrameVector velocity;

   private final YoFrameVector offsetYoFrameVector;

   protected Joint parentJoint;

   private KinematicPointUpdater kinematicPointUpdater;

   protected final Vector3d
      offsetFromCOM = new Vector3d(), wXr = new Vector3d(), v_point = new Vector3d();

   private RigidBodyTransform tempTransformFromWorldToJoint = new RigidBodyTransform();
   private Vector4d offsetPlus = new Vector4d();

   public KinematicPoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public KinematicPoint(String name, Vector3d offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, Vector3d offset, YoVariableRegistry registry)
   {
      this.name = name;

      position = new YoFramePoint(name + "_", "", ReferenceFrame.getWorldFrame(), registry);
      velocity = new YoFrameVector(name + "_d", "", ReferenceFrame.getWorldFrame(), registry);

      this.offsetYoFrameVector = new YoFrameVector(name + "off", "", ReferenceFrame.getWorldFrame(), registry);
      if (offset != null)
         offsetYoFrameVector.set(offset);
   }

   public void reset()
   {
      parentJoint = null;

      offsetYoFrameVector.set(0, 0, 0);

      offsetFromCOM.set(0, 0, 0);
      wXr.set(0, 0, 0);
      v_point.set(0, 0, 0);

      tempTransformFromWorldToJoint.setIdentity();
      offsetPlus.set(0, 0, 0, 0);

      position.set(0, 0, 0);
      velocity.set(0, 0, 0);
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
      return ("name: " + name + " x: " + position.getX() + ", y: " + position.getY() + ", z: " + position.getZ());
   }

   public void setOffsetJoint(double x, double y, double z)
   {
      this.offsetYoFrameVector.set(x, y, z);
   }

   public void setOffsetJoint(Vector3d newOffset)
   {
      this.offsetYoFrameVector.set(newOffset);
   }

   public void setOffsetWorld(Tuple3d offsetInWorld)
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

   }

   public void updatePointVelocity(Matrix3d R0_i, Vector3d comOffset, Vector3d v_i, Vector3d w_i)
   {
      this.getOffset(offsetFromCOM);
      offsetFromCOM.sub(comOffset);

      wXr.cross(w_i, offsetFromCOM);
      v_point.add(v_i, wXr);

      R0_i.transform(v_point);

      velocity.set(v_point);
   }

   protected void updatePointPosition(RigidBodyTransform t1)
   {
      if (kinematicPointUpdater != null)
      {
         // System.out.print(".");
         kinematicPointUpdater.updateKinematicPoint(this);
      }

      this.getOffset(temp);
      t1.transform(temp);

      position.set(temp);
   }

   public String getName()
   {
      return name;
   }

   public void getOffset(Tuple3d offsetToPack)
   {
      offsetYoFrameVector.get(offsetToPack);
   }

   public Vector3d getOffsetCopy()
   {
      Vector3d ret = new Vector3d();
      getOffset(ret);

      return ret;
   }

   public double getX()
   {
      return position.getX();
   }

   public double getY()
   {
      return position.getY();
   }

   public double getZ()
   {
      return position.getZ();
   }

   public double getXVelocity()
   {
      return velocity.getX();
   }

   public double getYVelocity()
   {
      return velocity.getY();
   }

   public double getZVelocity()
   {
      return velocity.getZ();
   }

   public void getPosition(Tuple3d positionToPack)
   {
      position.get(positionToPack);
   }

   public Point3d getPositionPoint()
   {
      Point3d pointToReturn = new Point3d();
      getPosition(pointToReturn);

      return pointToReturn;
   }

   public void getVelocity(Vector3d velocityToPack)
   {
      velocity.get(velocityToPack);
   }

   public Vector3d getVelocityVector()
   {
      Vector3d velocityToReturn = new Vector3d();
      velocity.get(velocityToReturn);

      return velocityToReturn;
   }

   public void setVelocity(Vector3d velocity)
   {
      this.velocity.set(velocity);
   }

   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   public YoFramePoint getYoPosition()
   {
      return position;
   }

   public YoFrameVector getYoVelocity()
   {
      return velocity;
   }
}
