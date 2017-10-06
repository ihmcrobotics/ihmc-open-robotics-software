package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.Axis;

public class UniversalJoint extends PinJoint
{
   /**
    *
    */
   private static final long serialVersionUID = 3428274232426974681L;
   private PinJoint joint2;

   public UniversalJoint(String jname1, String jname2, Vector3D offset, Robot rob, Axis firstAxis, Axis secondAxis)
   {
      super(jname1, offset, rob, firstAxis);

      joint2 = new PinJoint(jname2, new Vector3D(), rob, secondAxis);

      // super.addJoint(joint2); // This crashes.  Instead, add the joint manually:

      joint2.parentJoint = this;
      childrenJoints.add(joint2);

      // Set the child r_in value:

      joint2.physics.r_in.setX(0.0);
      joint2.physics.r_in.setY(0.0);
      joint2.physics.r_in.setZ(0.0);
   }
   
   public PinJoint getFirstJoint()
   {
      return this;
   }

   public PinJoint getSecondJoint()
   {
      return this.joint2;
   }

   @Override
   public void addJoint(Joint nextJoint)
   {
      joint2.addJoint(nextJoint);
   }

   @Override
   public void setLink(Link l)
   {
      // Set this joints real link to a null link and set the second Joints link to the given link...
      Link nullLink = new Link("null");    // smallPiece();
      nullLink.setMass(0.0);
      nullLink.setMomentOfInertia(0.0, 0.0, 0.0);
      nullLink.setComOffset(0.0, 0.0, 0.0);

      super.setLink(nullLink);
      joint2.setLink(l);
   }

   @Override
   public void addCameraMount(CameraMount mount)
   {
      joint2.addCameraMount(mount);
   }
   
   @Override
   public void addIMUMount(IMUMount mount)
   {
      joint2.addIMUMount(mount);
   }

   @Override
   public void addKinematicPoint(KinematicPoint point)
   {
      joint2.addKinematicPoint(point);
   }

   @Override
   public void addGroundContactPoint(GroundContactPoint point)
   {
      joint2.addGroundContactPoint(point);
   }

   @Override
   public void addExternalForcePoint(ExternalForcePoint point)
   {
      joint2.addExternalForcePoint(point);
   }

   /*
    * private Link smallPiece()
    * {
    * double BASE_H = 0.1, BASE_W = 0.1, BASE_L = 0.1;
    *
    * Link ret = new Link("small piece");
    * ret.translate(0.0,0.0,-BASE_H/2.0);
    * ret.addCube((float)BASE_L, (float)BASE_W, (float)BASE_H, YoAppearance.Red());
    * //ret.addPyramidCube(BASE_L, BASE_W, BASE_H, BASE_H, appearance);
    *
    * ret.identity();
    * ret.rotate(Math.PI/2.0,Link.Y);
    * ret.addCylinder(0.5,0.01, YoAppearance.Red());
    *
    * return ret;
    * }
    */


   public void setLimitStops(int axis, double q_min, double q_max, double k_limit, double b_limit)
   {
      if (axis == 1)
         super.setLimitStops(q_min, q_max, k_limit, b_limit);
      else if (axis == 2)
         joint2.setLimitStops(q_min, q_max, k_limit, b_limit);
   }

   public void setDamping(int axis, double b_damp)
   {
      if (axis == 1)
         super.setDamping(b_damp);
      else if (axis == 2)
         joint2.setDamping(b_damp);
   }

   @Override
   public void setDamping(double b_damp)
   {
      super.setDamping(b_damp);
      joint2.setDamping(b_damp);
   }

   public void setInitialState(double q1_init, double qd1_init, double q2_init, double qd2_init)
   {
      super.setInitialState(q1_init, qd1_init);
      joint2.setInitialState(q2_init, qd2_init);
   }

   @Override
   public void getState(double[] state)
   {
      state[0] = q.getDoubleValue();
      state[1] = qd.getDoubleValue();
      state[2] = joint2.q.getDoubleValue();
      state[3] = joint2.qd.getDoubleValue();
   }

   @Override
   public void getRotationToWorld(RotationMatrix rotation)
   {
      joint2.transformToNext.getRotation(rotation);
   }

   @Override
   public void getRotationToWorld(Quaternion rotation)
   {
      joint2.transformToNext.getRotation(rotation);
   }

   @Override
   public void getTranslationToWorld(Vector3D translation)
   {
      joint2.transformToNext.getTranslation(translation);
   }


}
