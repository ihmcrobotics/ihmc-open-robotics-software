package us.ihmc.avatar.arm;

/**
 * Preset arm configurations offered by the teleop UI. Each robot maps these to joint angles in its own
 * {@code *PresetArmConfigurations} class, reached via {@code DRCRobotModel#getPresetArmConfiguration}.
 * <p>
 * A robot that does not implement a constant here falls through its switch and returns an ALL-ZEROS
 * joint-angle array, which is then commanded to the arm. Add new constants at the END, and implement
 * them for the robots you intend to use them on.
 * </p>
 */
public enum PresetArmConfiguration
{
   HOME,
   N_POSE,
   INITIAL_SETUP,
   STAND_PREP,
   WIDE_ARMS,
   TUCKED_UP_ARMS,
   DOOR_AVOIDANCE,
   /** Arms in, elbows deeply flexed. Implemented for Alex only. */
   PRAY,
   /** Arms out front, forearms up to cradle a payload (the "static hold" load-carrying pose). Alex only. */
   STATIC_HOLD,
   ;

   public static final PresetArmConfiguration[] values = values();
}
