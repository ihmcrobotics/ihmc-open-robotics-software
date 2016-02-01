package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.SliderJoint;
import javax.vecmath.Vector3d;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.LinkGraphics;
import com.yobotics.simulationconstructionset.Link;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SimpleSecondOrderSystemModel extends Robot
{
  public final YoVariable t;

  public final double MASS = 2.0 * 1e-4;

  private final boolean SHOW_COORDINATE_SYSTEM = true;

  private final SliderJoint massJoint;

  public SimpleSecondOrderSystemModel()
  {
    super("RSEA");
    this.t = getVariable("t");

    massJoint = new SliderJoint("mass", new Vector3d(0.0, 0.0, 0.0), this, Joint.X);
    this.addRootJoint(massJoint);

    massJoint.setLink(getMassLink());
  }

  public void setTorque(double torque)
  {
    massJoint.setTau(torque);
  }

  public double getPosition()
  {
    return massJoint.getQ().val;
  }

  public double getVelocity()
  {
    return massJoint.getQD().val;
  }


  private Link getMassLink()
  {
    Link link = new Link("mass");

   LinkGraphics linkGraphics = new LinkGraphics();

   link.setLinkGraphics(linkGraphics);

   link.setMass(MASS);
   link.setComOffset(0.0, 0.0, 0.0);
   link.setMomentOfInertia(1.0, 1.0, 1.0);

   if (SHOW_COORDINATE_SYSTEM)
     linkGraphics.addCoordinateSystem(0.5);

   linkGraphics.addCube(0.1, 0.1, 0.1, YoAppearance.DarkRed());

   return link;

  }
}
