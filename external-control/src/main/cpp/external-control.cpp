#include "external-control.hpp"

namespace ihmc
{
    bool ExternalControlImpl::updateRobotState(const double current_time,
                                               const double* x_data, int x_rows,
                                               const double* u_data, int u_rows,
                                               const bool left_in_contact, const bool right_in_contact,
                                               const double* foot_locations, int foot_locations_rows,
                                               const int hardware_status)
    {
        return true;
    }

    bool ExternalControlImpl::getSolution(double* state_data_to_pack, int state_rows,
                         double* control_data_to_pack, int control_rows,
                         double* p_gains_to_pack, int p_gain_rows,
                         double* d_gains_to_pack, int d_gain_rows) const
    {
        return true;
    }
}
