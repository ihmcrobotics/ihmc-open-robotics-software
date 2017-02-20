package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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

   private final Point3D temp = new Point3D();

   private final YoFramePoint position;
   private final YoFrameVector velocity;
   private final YoFrameVector angularVelocity;

   private final YoFrameVector offsetYoFrameVector;

   protected Joint parentJoint;

   private KinematicPointUpdater kinematicPointUpdater;

   private final YoVariableRegistry registry;

   protected final Vector3D
      offsetFromCOM = new Vector3D(), wXr = new Vector3D(), v_point = new Vector3D();

   private RigidBodyTransform tempTransformFromWorldToJoint = new RigidBodyTransform();
   private Vector4D offsetPlus = new Vector4D();

   public KinematicPoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public KinematicPoint(String name, Vector3D offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, Vector3D offset, YoVariableRegistry registry)
   {
      this.name = name;
      this.registry = registry;

      position = new YoFramePoint(name + "_", "", ReferenceFrame.getWorldFrame(), registry);
      velocity = new YoFrameVector(name + "_d", "", ReferenceFrame.getWorldFrame(), registry);
      angularVelocity = new YoFrameVector(name + "_w", "", ReferenceFrame.getWorldFrame(), registry);

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
      angularVelocity.set(0, 0, 0);
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

   public void setOffsetJoint(Vector3D newOffset)
   {
      this.offsetYoFrameVector.set(newOffset);
   }

   public void setOffsetWorld(Tuple3DBasics offsetInWorld)
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
      this.position.set(x, y, z);
   }

   public void updatePointVelocity(RotationMatrix R0_i, Vector3D comOffset, Vector3D v_i, Vector3D w_i)
   {
      this.getOffset(offsetFromCOM);
      offsetFromCOM.sub(comOffset);

      wXr.cross(w_i, offsetFromCOM);
      v_point.add(v_i, wXr);

      R0_i.transform(v_point);

      velocity.set(v_point);
      angularVelocity.set(w_i);
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

   public void getOffset(Tuple3DBasics offsetToPack)
   {
      offsetYoFrameVector.get(offsetToPack);
   }

   public Vector3D getOffsetCopy()
   {
      Vector3D ret = new Vector3D();
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

   public void getPosition(Tuple3DBasics positionToPack)
   {
      position.get(positionToPack);
   }

   public Point3D getPositionPoint()
   {
      Point3D pointToReturn = new Point3D();
      getPosition(pointToReturn);

      return pointToReturn;
   }

   public void getVelocity(Vector3D velocityToPack)
   {
      velocity.get(velocityToPack);
   }

   public Vector3D getVelocityVector()
   {
      Vector3D velocityToReturn = new Vector3D();
      velocity.get(velocityToReturn);

      return velocityToReturn;
   }

   public void setVelocity(Vector3D velocity)
   {
      this.velocity.set(velocity);
   }

   public void setAngularVelocity(Vector3D angularVelocity)
   {
      this.velocity.set(velocity);
   }

   public void setPosition(Point3D position)
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

   public YoFrameVector getYoAngularVelocity()
   {
      return angularVelocity;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
