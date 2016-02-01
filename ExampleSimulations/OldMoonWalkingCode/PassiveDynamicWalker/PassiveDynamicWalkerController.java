package us.ihmc.moonwalking.models.PassiveDynamicWalker;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

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
public class PassiveDynamicWalkerController implements RobotController
{
  private enum States{SWING_KNEE_UNLOCK, SWING_KNEE_LOCK, DONE};

  private boolean stanceLegLeft = true;
  private final YoVariableRegistry yoVariableRegistry;
  private final PassiveDynamicWalkerRobot passiveDynamicWalker;
  private YoVariable q_kneeLeft;
  private YoVariable q_hipLeft;
  private YoVariable q_hipRight;
  private YoVariable q_kneeRight;
  private YoVariable qd_kneeLeft;
  private YoVariable qd_hipLeft;
  private YoVariable tau_kneeLeft;
  private YoVariable qd_hipRight;
  private YoVariable qd_kneeRight;
  private YoVariable tau_kneeRight;

  private final YoVariable passiveWalkerState;

  private final YoVariable kKneeLock, bKneeLock;

  public PassiveDynamicWalkerController(PassiveDynamicWalkerRobot passiveDynamicWalker)
  {
    yoVariableRegistry = new YoVariableRegistry("controller");
    this.passiveDynamicWalker = passiveDynamicWalker;
    this.passiveWalkerState = new YoVariable("passiveWalkerState", States.values(), yoVariableRegistry);

    this.kKneeLock = new YoVariable("kKneeLock", yoVariableRegistry);
    this.bKneeLock = new YoVariable("bKneeLock", yoVariableRegistry);

    this.q_kneeLeft = passiveDynamicWalker.getVariable("q_kneeLeft");
    this.q_kneeRight = passiveDynamicWalker.getVariable("q_kneeRight");
    this.q_hipLeft = passiveDynamicWalker.getVariable("q_hipLeft");
    this.q_hipRight = passiveDynamicWalker.getVariable("q_hipRight");
    this.qd_hipRight = passiveDynamicWalker.getVariable("qd_hipRight");
    this.qd_hipLeft = passiveDynamicWalker.getVariable("qd_hipLeft");
    this.qd_kneeLeft = passiveDynamicWalker.getVariable("qd_kneeLeft");
    this.tau_kneeLeft = passiveDynamicWalker.getVariable("tau_kneeLeft");
    this.qd_kneeRight = passiveDynamicWalker.getVariable("qd_kneeRight");
    this.tau_kneeRight = passiveDynamicWalker.getVariable("tau_kneeRight");

    passiveWalkerState.set(States.SWING_KNEE_UNLOCK);

    kKneeLock.val =  20.0;
    bKneeLock.val = 2.0;
  }

  public void doControl()
  {
    if(stanceLegLeft)
    {
      tau_kneeLeft.val = kKneeLock.val * (0.0 - q_kneeLeft.val) - bKneeLock.val * qd_kneeLeft.val;
    }
    else
    {
      tau_kneeRight.val = kKneeLock.val * (0.0 - q_kneeRight.val) - bKneeLock.val * qd_kneeRight.val;
    }

// Transition Conditions
    switch((States) passiveWalkerState.getEnumValue())
    {
      case SWING_KNEE_UNLOCK:
      {
        if(stanceLegLeft)
        {
          if(q_hipRight.val<-0.15 && Math.abs(q_kneeRight.val)<0.03)
          {
            passiveWalkerState.set(States.SWING_KNEE_LOCK);
          }
        }
        else
        {
          if(q_hipLeft.val<-0.15 && Math.abs(q_kneeLeft.val)<0.03)
          {
            passiveWalkerState.set(States.SWING_KNEE_LOCK);
          }
        }
        break;
      }

      case SWING_KNEE_LOCK:
      {
        if(stanceLegLeft)
        {
          for(int i=1; i<passiveDynamicWalker.groundContactpoints; i=i+2)
          {
            YoVariable gc_heel = passiveDynamicWalker.getVariable("gc_heel_"+i+"_right_fs");
            if(gc_heel.val == 1.0)
            {
              passiveWalkerState.set(States.DONE);
            }
          }
        }
        else
        {
          for(int i=0; i<passiveDynamicWalker.groundContactpoints; i=i+2)
          {
            YoVariable gc_heel = passiveDynamicWalker.getVariable("gc_heel_"+i+"_left_fs");
            if(gc_heel.val == 1.0)
            {
              passiveWalkerState.set(States.DONE);
            }
          }
        }
        break;
      }

      case DONE:
      {
        passiveWalkerState.set(States.SWING_KNEE_UNLOCK);
        break;
      }
    }

//    Do the actions
    switch((States) passiveWalkerState.getEnumValue())
        {
          case SWING_KNEE_UNLOCK:
          {
            break;
          }

          case SWING_KNEE_LOCK:
          {
            if(!stanceLegLeft)
            {
              tau_kneeLeft.val = kKneeLock.val * (0.0 - q_kneeLeft.val) - bKneeLock.val * qd_kneeLeft.val;
            }
            else
            {
              tau_kneeRight.val = kKneeLock.val * (0.0 - q_kneeRight.val) - bKneeLock.val * qd_kneeRight.val;
            }
            break;
          }

          case DONE:
          {
            if(stanceLegLeft)
            {
              stanceLegLeft = false;
            }
            else
            {
              stanceLegLeft = true;
            }
            break;
          }
    }
 }

   public YoVariableRegistry getYoVariableRegistry()
   {
     return yoVariableRegistry;
   }

}



