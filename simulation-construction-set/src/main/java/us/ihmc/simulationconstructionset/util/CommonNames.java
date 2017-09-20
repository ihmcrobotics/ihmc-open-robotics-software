package us.ihmc.simulationconstructionset.util;

/**
 * Centralized location for the names related to contols
 *
 * <p>
 * Nomenqlature:<br>
 * <ul>
 * <li>q_ = position.  from Lagrangian mechanics</li>
 * <li>qd_ = velocity.  deriviative of position</li>
 * <li>STUFF_d = desired value of STUFF</li>
 * <li>_k = gain</li>
 * </ul>
 * </p>
 *
 */
public enum CommonNames
{
   q_d("_q_d"),
   qd_d("_qd_d"),
   f_d("_f_d"),
   k_q_p("_k_q_p"),
   k_q_i("_k_q_i"),
   k_qd_p("_k_qd_p"),
   k_f_p("_k_f_p"),
   ff_qd("_ff_qd"),
   ff_qd_d("_ff_qd_d"),
   ff_f_d("_ff_f_d"),
   ff_const("_ff_const"),
   a_k_q_p("_a_k_q_p"),
   a_k_q_i("_a_k_q_i"),
   a_k_qd_p("_a_k_qd_p"),
   a_k_f_p("_a_k_f_p"),
   a_ff_qd("_a_ff_qd"),
   a_ff_qd_d("_a_ff_qd_d"),
   a_ff_f_d("_a_ff_f_d"),
   qerr_int_abs_max("qerr_int_abs_max"),
   kp("_kp"),
   kd("_kd"),
   current("_i"),
   doIHMCControlRatio("doIHMCControlRatio");

   private String name;

   CommonNames(String name)
   {
      this.name = name;
   }

   @Override
   public String toString()
   {
      return name;
   }
}
