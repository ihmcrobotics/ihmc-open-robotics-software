package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.RobotController;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import com.yobotics.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;

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
public class SimpleSecondOrderSystemController implements RobotController
{
  private static final double HERTZ_TO_RADIANS = 2.0 * Math.PI;
  private final SimpleSecondOrderSystemModel simpleSecondOrderSystemModel;
  private final YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("massController");


  private final YoVariable wn = new YoVariable("wn", "Natural Freq", yoVariableRegistry);
  private final YoVariable zeta = new YoVariable("zeta", "Damping ratio", yoVariableRegistry);

  private final YoVariable k = new YoVariable("k", "Spring constant", yoVariableRegistry);
  private final YoVariable b = new YoVariable("b", "Damping", yoVariableRegistry);

  private final YoVariable tauApplied = new YoVariable("tauApplied", "Applied force", yoVariableRegistry);

  private final YoFunctionGenerator functionGenerator = new YoFunctionGenerator("functionGenerator", yoVariableRegistry);

  private final YoVariable time;

  public SimpleSecondOrderSystemController(SimpleSecondOrderSystemModel simpleSecondOrderSystemModel)
  {
    this.simpleSecondOrderSystemModel = simpleSecondOrderSystemModel;

    this.time = simpleSecondOrderSystemModel.t;

    wn.val = 10.0 * HERTZ_TO_RADIANS;
    zeta.val = 0.7;

    b.val = 2.0 * zeta.val * wn.val * simpleSecondOrderSystemModel.MASS;
    k.val = wn.val * wn.val * simpleSecondOrderSystemModel.MASS;

//    setSine();
    setChrip();
  }

  private void setChrip()
  {
    functionGenerator.setPauseTime(0.25);
    functionGenerator.setAmplitude(100.0);
    functionGenerator.setChirpRate(5.0); //40.0);
    functionGenerator.setResetTime(20.0);

    functionGenerator.setMode(YoFunctionGeneratorMode.CHIRP);
  }

  private void setSine()
  {
    functionGenerator.setAmplitude(10.0);
    functionGenerator.setFrequency(wn.val/HERTZ_TO_RADIANS);
    functionGenerator.setMode(YoFunctionGeneratorMode.SINE);
  }

  public void doControl()
  {
     tauApplied.val = functionGenerator.getValue(time.val);

     double springForce = k.val * simpleSecondOrderSystemModel.getPosition();
     double dampingForce = b.val * simpleSecondOrderSystemModel.getVelocity();


     double totalForce = tauApplied.val - dampingForce - springForce;
     simpleSecondOrderSystemModel.setTorque(totalForce);
  }

  public YoVariableRegistry getYoVariableRegistry()
  {
    return yoVariableRegistry;
  }
}
